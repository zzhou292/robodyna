"""Sample and stop one Linux session created by this runner.

The direct child stays unreaped until cleanup finishes. Its PID/start time pins
the session identity while descendants may create independent process groups.
This is sampled supervision, not containment: a deliberate setsid() escape is
outside this profile. No command lines or environments are read.
"""

from dataclasses import dataclass
import os
from pathlib import Path
import signal
import time


@dataclass(frozen=True)
class ProcessRow:
    pid: int
    ppid: int
    pgid: int
    sid: int
    start_ticks: int
    state: str
    rss_pages: int
    cpu_ticks: int
    threads: int

    @property
    def identity(self):
        return self.pid, self.start_ticks

    @property
    def live(self):
        return self.state not in ('Z', 'X', 'x')


def read_process(pid, proc_root=Path('/proc')):
    try:
        value = (proc_root / str(pid) / 'stat').read_text()
    except (FileNotFoundError, ProcessLookupError):
        return None
    # comm can contain spaces and parentheses; its final ')' precedes state.
    prefix, suffix = value.rsplit(')', 1)
    fields = suffix.split()
    if int(prefix.split('(', 1)[0]) != pid:
        raise ValueError('process stat PID mismatch')
    return ProcessRow(pid, int(fields[1]), int(fields[2]), int(fields[3]),
                      int(fields[19]), fields[0], max(0, int(fields[21])),
                      int(fields[11]) + int(fields[12]), int(fields[17]))


def process_rows(proc_root=Path('/proc')):
    for entry in proc_root.iterdir():
        if not entry.name.isdigit():
            continue
        try:
            row = read_process(int(entry.name), proc_root)
        except (PermissionError, ValueError, IndexError):
            # Unreadable unrelated /proc entries must not prevent monitoring.
            continue
        if row is not None:
            yield row


def preflight():
    required = (getattr(os, 'pidfd_open', None),
                getattr(signal, 'pidfd_send_signal', None),
                getattr(os, 'waitid', None), getattr(os, 'WNOWAIT', None))
    if not all(required):
        raise RuntimeError('owned-session monitoring requires Linux pidfd and waitid(WNOWAIT)')
    if signal.getsignal(signal.SIGCHLD) != signal.SIG_DFL:
        raise RuntimeError('owned-session monitoring requires unreaped child status')
    try:
        # This process cannot be its own child. Exercise the supported flags
        # without inspecting or consuming any other child's exit status.
        os.waitid(os.P_PID, os.getpid(), os.WEXITED | os.WNOHANG | os.WNOWAIT)
    except ChildProcessError:
        pass
    descriptor = os.pidfd_open(os.getpid())
    try:
        signal.pidfd_send_signal(descriptor, 0)
    finally:
        os.close(descriptor)


class OwnedSession:
    def __init__(self, process, proc_root=Path('/proc')):
        self.process = process
        self.proc_root = proc_root
        self.leader = read_process(process.pid, proc_root)
        if (self.leader is None or self.leader.ppid != os.getpid()
                or self.leader.sid != process.pid or self.leader.pgid != process.pid):
            raise RuntimeError('launched child does not authenticate a new owned session')
        self.record = dict(policy='owned_session_v1', session_id=self.leader.sid,
                           leader_pid=self.leader.pid,
                           leader_start_ticks=self.leader.start_ticks,
                           cleanup='not_requested')
        self.cleanup_error = None

    def _check_leader(self):
        current = read_process(self.leader.pid, self.proc_root)
        if (current is None or current.identity != self.leader.identity
                or current.sid != self.leader.sid or current.ppid != os.getpid()):
            raise RuntimeError('owned session leader identity was lost before cleanup')

    def members(self):
        self._check_leader()
        rows = [row for row in process_rows(self.proc_root)
                if row.sid == self.leader.sid]
        self._check_leader()
        return rows

    def usage(self):
        rows = self.members()
        return dict(rss_bytes=sum(row.rss_pages for row in rows) * os.sysconf('SC_PAGE_SIZE'),
                    live_cpu_ticks=sum(row.cpu_ticks for row in rows),
                    threads=sum(row.threads for row in rows))

    def exited(self):
        self._check_leader()
        # Popen.poll() would reap the leader and permit numeric PID/SID reuse.
        return os.waitid(os.P_PID, self.process.pid,
                         os.WEXITED | os.WNOHANG | os.WNOWAIT) is not None

    def _signal(self, row, signum):
        self._check_leader()
        try:
            descriptor = os.pidfd_open(row.pid)
        except ProcessLookupError:
            return False
        try:
            current = read_process(row.pid, self.proc_root)
            if (current is None or current.identity != row.identity
                    or current.sid != self.leader.sid or not current.live):
                return False
            # The pidfd targets this exact process even if it exits and the
            # numeric PID is reused between the check and signal delivery.
            signal.pidfd_send_signal(descriptor, signum)
            return True
        except ProcessLookupError:
            return False
        finally:
            os.close(descriptor)

    def _drain(self, signum, seconds):
        deadline = time.monotonic() + seconds
        signaled = set()
        while True:
            live = [row for row in self.members() if row.live]
            if not live:
                # A fresh scan closes the ordinary enumeration/exit race;
                # zombies cannot create more children and need no signals.
                live = [row for row in self.members() if row.live]
                if not live:
                    return True
            # Keep only currently observed identities, not an unbounded log.
            signaled.intersection_update(row.identity for row in live)
            for row in live:
                if row.identity not in signaled and self._signal(row, signum):
                    signaled.add(row.identity)
            if time.monotonic() >= deadline:
                return False
            time.sleep(0.05)

    def stop(self):
        if self.record['cleanup'] == 'complete':
            return
        if self.cleanup_error is not None:
            raise RuntimeError(self.cleanup_error)
        try:
            self.record['cleanup'] = 'term_requested'
            if not self._drain(signal.SIGTERM, 2):
                self.record['cleanup'] = 'kill_requested'
                if not self._drain(signal.SIGKILL, 2):
                    raise RuntimeError('owned session cleanup timed out with live members')
            # The leader is reaped only after all same-session members stopped.
            self.process.wait()
            self.record['cleanup'] = 'complete'
        except (OSError, RuntimeError, ValueError) as error:
            self.cleanup_error = str(error)
            self.record['cleanup'] = 'failed'
            raise
