"""Optional, create-only stop request for a guarded command's own controller.

No metrics or clock are sampled here. The runner supplies its existing sample
and elapsed time, and remains responsible for hard limits and group cleanup.
"""

import json
import os
from pathlib import Path


GPU_GROWTH_REASON = 'total GPU memory growth exceeded job allowance'


class CooperativeStop:
    def __init__(self, path, grace_seconds):
        self.path = Path(path).absolute()
        self.grace_seconds = grace_seconds
        self.deadline = None
        self.record = dict(stop_file=str(self.path), grace_seconds=grace_seconds,
                           outcome='not_requested')

    def preflight(self, report_path, lock_path):
        if self.path.resolve() in {report_path.resolve(), lock_path.resolve()}:
            raise RuntimeError('cooperative stop file aliases the report or lock')
        if os.path.lexists(self.path):
            raise RuntimeError('cooperative stop file must not already exist')
        if not self.path.parent.is_dir():
            raise RuntimeError('cooperative stop file parent directory is absent')

    def request(self, elapsed, sample):
        if self.deadline is not None:
            return
        self.record.update(trigger=GPU_GROWTH_REASON,
                           trigger_sample=dict(sample),
                           request_elapsed_seconds=elapsed,
                           request_created=False,
                           request_completed=False,
                           outcome='request_failed')
        # Existence is the existing controller's signal. Never truncate another
        # file or follow a final symlink, including one created after preflight.
        with self.path.open('x') as stream:
            self.record['request_created'] = True
            json.dump(dict(reason=GPU_GROWTH_REASON,
                           elapsed_seconds=elapsed), stream)
            stream.write('\n')
            stream.flush()
            os.fsync(stream.fileno())
        self.deadline = elapsed + self.grace_seconds
        self.record.update(outcome='requested',
                           request_completed=True,
                           grace_deadline_elapsed_seconds=self.deadline)

    def expired(self, elapsed):
        return self.deadline is not None and elapsed >= self.deadline

    def exited(self, elapsed, returncode):
        if self.deadline is not None:
            self.record.update(outcome='cooperative_exit' if returncode == 0 else 'command_failed',
                               completion_elapsed_seconds=elapsed,
                               child_exit_code=returncode)

    def forced(self, elapsed, reason):
        if self.record['outcome'] != 'not_requested':
            if self.record['outcome'] != 'request_failed':
                self.record['outcome'] = 'forced_stop'
            self.record.update(termination_requested_elapsed_seconds=elapsed,
                               forced_reason=reason)
