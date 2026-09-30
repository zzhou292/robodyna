"""Tiny real Linux children exercise separate process groups; no GPU tools."""

import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import unittest

from bounded_session import read_process


RUNNER = Path(__file__).with_name('run_bounded.py')
CHILD = r'''
import json, os, pathlib, signal, sys, time
root, mode, size = pathlib.Path(sys.argv[1]), sys.argv[2], int(sys.argv[3])
child = os.fork()
if child == 0:
    os.setpgid(0, 0)
    if mode == 'ignore':
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
    else:
        def term(signum, frame):
            (root / 'term').write_text('received')
            raise SystemExit(0)
        signal.signal(signal.SIGTERM, term)
    allocation = bytearray(size * 1024 * 1024)
    fields = pathlib.Path('/proc/self/stat').read_text().rsplit(')', 1)[1].split()
    record = dict(pid=os.getpid(), parent=os.getppid(), pgid=os.getpgrp(),
                  sid=os.getsid(0), start_ticks=int(fields[19]))
    (root / 'ready.tmp').write_text(json.dumps(record))
    (root / 'ready').write_bytes((root / 'ready.tmp').read_bytes())
    while True:
        time.sleep(.1)
while not (root / 'ready').exists():
    time.sleep(.005)
if mode == 'parent_exit':
    raise SystemExit(2)
os.waitpid(child, 0)
'''


class RealSessionTests(unittest.TestCase):
    def setUp(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.root = Path(directory.name)
        self.report = self.root / 'report.json'
        self.helper = self.root / 'child.py'
        self.helper.write_text(CHILD)

    def start(self, mode='normal', size=1, *options):
        output = (self.root / 'output.log').open('w')
        self.addCleanup(output.close)
        child = subprocess.Popen(
            [sys.executable, str(RUNNER), '--report', str(self.report),
             '--cpus', '1', '--min-available-gib', '0', '--max-rss-gib', '.25',
             '--timeout', '5', *options, '--', sys.executable,
             str(self.helper), str(self.root), mode, str(size)],
            stdout=output, stderr=output)
        self.addCleanup(self.cleanup, child)
        return child

    def ready(self, runner):
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            try:
                result = json.loads((self.root / 'ready').read_text())
                self.assertNotEqual(result['pgid'], result['sid'])
                self.assertEqual(result['pgid'], result['pid'])
                return result
            except (FileNotFoundError, json.JSONDecodeError):
                if runner.poll() is not None:
                    self.fail((self.root / 'output.log').read_text())
                time.sleep(.01)
        self.fail('separate-group child did not become ready')

    def cleanup(self, runner):
        if runner.poll() is None:
            runner.terminate()
            try:
                runner.wait(timeout=6)
            except subprocess.TimeoutExpired:
                runner.kill()
                runner.wait(timeout=3)
        # Test-failure cleanup is restricted to the exact published fixture PID.
        path = self.root / 'ready'
        if path.exists():
            value = json.loads(path.read_text())
            row = read_process(value['pid'])
            if row is not None and row.live and row.start_ticks == value['start_ticks']:
                try:
                    descriptor = os.pidfd_open(row.pid)
                except ProcessLookupError:
                    return
                try:
                    fresh = read_process(row.pid)
                    if fresh is not None and fresh.identity == row.identity and fresh.sid == value['sid']:
                        signal.pidfd_send_signal(descriptor, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                finally:
                    os.close(descriptor)

    def assert_stopped(self, record):
        row = read_process(record['pid'])
        self.assertTrue(row is None or row.identity != (record['pid'], record['start_ticks'])
                        or not row.live, 'owned descendant remains live')

    def result(self, runner, exit_code=125):
        self.assertEqual(runner.wait(timeout=9), exit_code,
                         (self.root / 'output.log').read_text())
        result = json.loads(self.report.read_text())
        self.assertEqual(result['process_scope']['policy'], 'owned_session_v1')
        self.assertEqual(result['process_scope']['cleanup'], 'complete')
        self.assertNotIn('cleanup_error', result)
        return result

    def test_separate_group_allocation_hits_original_rss_limit(self):
        runner = self.start('normal', 64, '--max-rss-gib', '.055')
        record = self.ready(runner)
        result = self.result(runner)
        self.assertEqual(result['reason'], 'job RSS exceeded budget')
        self.assertGreater(result['peak_sampled_rss_bytes'], .055 * 1024 ** 3)
        self.assertEqual(result['limits']['max_rss_gib'], .055)
        self.assert_stopped(record)

    def test_timeout_terms_all_groups_and_leaves_unrelated_session_running(self):
        unrelated = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(30)'],
                                     start_new_session=True)
        try:
            runner = self.start('normal', 1, '--timeout', '.6')
            record = self.ready(runner)
            result = self.result(runner)
            self.assertEqual(result['reason'], 'command timeout')
            self.assertEqual((self.root / 'term').read_text(), 'received')
            self.assert_stopped(record)
            self.assertIsNone(unrelated.poll())
            self.assertNotEqual(os.getsid(unrelated.pid), record['sid'])
        finally:
            if unrelated.poll() is None:
                unrelated.terminate()
            unrelated.wait(timeout=3)

    def test_runner_term_cleans_separate_group(self):
        runner = self.start()
        record = self.ready(runner)
        runner.send_signal(signal.SIGTERM)
        result = self.result(runner)
        self.assertEqual(result['reason'], 'runner interrupted by signal 15')
        self.assert_stopped(record)
        self.assertTrue((self.root / 'term').exists())

    def test_ignored_term_gets_kill_after_grace(self):
        runner = self.start('ignore', 1, '--timeout', '.5')
        record = self.ready(runner)
        result = self.result(runner)
        self.assertEqual(result['reason'], 'command timeout')
        self.assertGreaterEqual(result['elapsed_seconds'], 2.5)
        self.assert_stopped(record)

    def test_lock_remains_held_during_exception_cleanup(self):
        runner = self.start('ignore', 1, '--timeout', '.5')
        record = self.ready(runner)
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline:
            parent = read_process(record['parent'])
            if parent is not None and parent.state == 'Z':
                break  # TERM delivered; ignored child keeps cleanup in grace.
            time.sleep(.01)
        else:
            self.fail('cleanup did not retain its exited leader')
        marker = self.root / 'contender-entered'
        contender = subprocess.run(
            [sys.executable, str(RUNNER), '--report', str(self.root / 'contender.json'),
             '--cpus', '1', '--min-available-gib', '0', '--', sys.executable, '-c',
             f'import pathlib; pathlib.Path({str(marker)!r}).touch()'],
            capture_output=True, text=True, timeout=1)
        self.assertEqual(contender.returncode, 125, contender.stderr)
        self.assertFalse(marker.exists())
        self.assertEqual(json.loads((self.root / 'contender.json').read_text())['samples'], [])
        self.result(runner)
        self.assert_stopped(record)

    def test_natural_parent_exit_two_preserved_while_orphan_group_is_cleaned(self):
        runner = self.start('parent_exit')
        record = self.ready(runner)
        result = self.result(runner, 2)
        self.assertEqual(result['status'], 'command_failed')
        self.assert_stopped(record)
        self.assertTrue((self.root / 'term').exists())

    def test_fast_natural_exit_preserves_zero_and_nonzero_codes(self):
        for code in (0, 2):
            with self.subTest(code=code):
                result = subprocess.run(
                    [sys.executable, str(RUNNER), '--report', str(self.report),
                     '--cpus', '1', '--min-available-gib', '0', '--',
                     sys.executable, '-c', f'raise SystemExit({code})'],
                    capture_output=True, text=True, timeout=5)
                self.assertEqual(result.returncode, code, result.stderr)
                report = json.loads(self.report.read_text())
                self.assertEqual(report['process_scope']['cleanup'], 'complete')


if __name__ == '__main__':
    unittest.main()
