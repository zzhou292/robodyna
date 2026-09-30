"""Optional bounded compute-process diagnostics for the workstation guard.

Whole-device memory remains the sole GPU guard input. Compute-only observations
are sampled at different times from /proc and whole-device telemetry. They do
not account for graphics, driver/context allocations, MIG/MPS accounting or
unreported process memory, and cannot prove attribution of a memory change.
"""

from collections import deque
from datetime import datetime, timezone
from itertools import islice
import time

if __package__:
    from .bounded_gpu_query import OUTPUT_CAP, QUERY_TIMEOUT_SECONDS, query_compute_apps
else:
    from bounded_gpu_query import OUTPUT_CAP, QUERY_TIMEOUT_SECONDS, query_compute_apps

MIB = 1024 * 1024
MAX_PROCESS_ROWS = 64
MAX_IDENTITY_ROWS = 4096
MAX_SNAPSHOTS = 128
SCOPE = ('compute apps only; non-atomic sampled membership; graphics, driver, '
         'MIG/MPS and unavailable memory are not attributed; no leak or '
         'whole-device growth attribution follows from these sums')


def parse_compute_apps(data):
    """Return a capped roster and explicit complete-query row count.

    A missing/N/A value is never zero. Duplicate PIDs are ambiguous accounting,
    so reject their inventory rather than double counting them. Output itself
    was capped by the query reader before decoding or allocating row strings.
    """
    if len(data) > OUTPUT_CAP:
        raise ValueError('compute query exceeds byte cap')
    rows, seen, count = [], set(), 0
    for line in data.decode('ascii').splitlines():
        if not line.strip():
            continue
        fields = [value.strip() for value in line.split(',')]
        if len(fields) != 2 or not fields[0].isascii() or not fields[0].isdecimal():
            raise ValueError('invalid compute-process row')
        pid = int(fields[0])
        if not 0 < pid <= 2147483647 or pid in seen:
            raise ValueError('invalid or duplicate compute-process PID')
        seen.add(pid)
        memory = fields[1]
        if memory in ('N/A', '[N/A]', 'Not Supported', '[Not Supported]', '-'):
            used = None
        elif memory.isascii() and memory.isdecimal() and int(memory) <= (2**64 - 1) // MIB:
            used = int(memory) * MIB
        else:
            raise ValueError('invalid compute-process memory')
        count += 1
        if len(rows) < MAX_PROCESS_ROWS:
            rows.append(dict(pid=pid, used_bytes=used))
    return rows, count


def _bounded_identities(rows):
    result = {}
    for row in islice(rows, MAX_IDENTITY_ROWS):
        if row.live:
            result[row.pid] = row
    return result


def _same_process(before, after):
    return (before is not None and after is not None and before.live and after.live
            and before.identity == after.identity and before.sid == after.sid)


def classify_processes(rows, owned_before, owned_after, all_before, read_after, owned_sid):
    """Authenticate membership on both sides of the non-atomic GPU query."""
    totals = {kind: dict(processes=0, known_used_bytes=0, unknown_memory_processes=0)
              for kind in ('owned', 'other', 'unknown')}
    result = []
    for value in rows:
        pid = value['pid']
        try:
            after = read_after(pid)
        except (OSError, ValueError, IndexError):
            after = None
        before = all_before.get(pid)
        first_owned, last_owned = owned_before.get(pid), owned_after.get(pid)
        if (_same_process(first_owned, last_owned)
                and first_owned.sid == owned_sid and last_owned.sid == owned_sid
                and _same_process(last_owned, after)):
            kind = 'owned'
        elif (_same_process(before, after) and before.sid != owned_sid
              and after.sid != owned_sid
              and pid not in owned_before and pid not in owned_after):
            kind = 'other'
        else:
            kind = 'unknown'
        row = dict(value, membership=kind,
                   process_start_ticks=after.start_ticks if kind != 'unknown' else None)
        result.append(row)
        group = totals[kind]
        group['processes'] += 1
        if value['used_bytes'] is None:
            group['unknown_memory_processes'] += 1
        else:
            group['known_used_bytes'] += value['used_bytes']
    return result, totals


class GpuProcessDiagnostics:
    """Retain an initial snapshot, bounded chronological tail and stop snapshot."""
    def __init__(self, index, started):
        self.index, self.started = index, started
        self.initial = None
        self.tail = deque(maxlen=MAX_SNAPSHOTS)
        self.total = 0
        self.before_stop = None

    def _capture(self, session):
        # Each bounded snapshot authenticates the unreaped leader on both
        # sides of enumeration. Missing identities remain explicitly unknown.
        before, before_complete = session.process_snapshot(MAX_IDENTITY_ROWS)
        all_before = _bounded_identities(before)
        sid = session.record['session_id']
        owned_before = {pid: row for pid, row in all_before.items() if row.sid == sid}
        query = query_compute_apps(self.index)
        if query.status != 'ok':
            return dict(status=query.status, query_returncode=query.returncode,
                        rows=[], row_count=None, rows_complete=False, totals=None)
        rows, count = parse_compute_apps(query.data)
        after, after_complete = session.process_snapshot(MAX_IDENTITY_ROWS)
        all_after = _bounded_identities(after)
        owned_after = {pid: row for pid, row in all_after.items() if row.sid == sid}
        rows, totals = classify_processes(
            rows, owned_before, owned_after, all_before,
            lambda pid: all_after.get(pid), sid)
        return dict(status='ok', query_returncode=query.returncode,
                    rows=rows, row_count=count, retained_rows=len(rows),
                    omitted_rows=count - len(rows), rows_complete=count == len(rows),
                    membership_before_complete=before_complete,
                    membership_after_complete=after_complete,
                    totals=totals, totals_scope='retained compute rows only; known memory subtotal')

    def sample(self, session, trigger, gpu, gpu_poll_elapsed_seconds):
        """Ordinary diagnostic errors are informational; interruptions propagate."""
        began = time.monotonic()
        snapshot = dict(trigger=trigger, started_utc=datetime.now(timezone.utc).isoformat(),
                        started_elapsed_seconds=began - self.started,
                        gpu_poll_elapsed_seconds=gpu_poll_elapsed_seconds,
                        whole_device_used_bytes_at_guard_poll=gpu['used_bytes'])
        try:
            snapshot.update(self._capture(session))
        except Exception as error:
            snapshot.update(status='diagnostic_error', rows=[], row_count=None,
                            rows_complete=False, totals=None,
                            error_type=type(error).__name__, error=str(error)[:256])
        snapshot['finished_elapsed_seconds'] = time.monotonic() - self.started
        snapshot['finished_utc'] = datetime.now(timezone.utc).isoformat()
        self.total += 1
        snapshot['sequence'] = self.total
        if self.initial is None:
            self.initial = snapshot
        self.tail.append(snapshot)
        if trigger == 'before_gpu_growth_stop':
            self.before_stop = snapshot

    def document(self):
        return dict(schema='bounded_gpu_process_diagnostics_v1', gpu=self.index,
                    scope=SCOPE, process_row_cap=MAX_PROCESS_ROWS,
                    identity_row_cap=MAX_IDENTITY_ROWS, snapshot_tail_cap=MAX_SNAPSHOTS,
                    query_output_byte_cap=OUTPUT_CAP, query_timeout_seconds=QUERY_TIMEOUT_SECONDS,
                    query_cleanup_timeout_seconds=1.0,
                    total_snapshots=self.total, tail_snapshots=len(self.tail),
                    tail_evicted_snapshots=max(0, self.total - len(self.tail)),
                    initial_snapshot=self.initial, snapshots=list(self.tail),
                    before_gpu_growth_stop=self.before_stop)
