"""Small subprocess regressions for the workstation guard itself."""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import unittest

RUNNER = Path(__file__).with_name('run_bounded.py')


class BoundedRunnerTests(unittest.TestCase):
    def run_job(self, command, *options):
        with tempfile.TemporaryDirectory() as directory:
            report = Path(directory) / 'report.json'
            result = subprocess.run(
                [sys.executable, str(RUNNER), '--report', str(report),
                 '--min-available-gib', '0', *options, '--', *command],
                capture_output=True, text=True, timeout=15)
            return result, json.loads(report.read_text())

    def test_affinity_and_thread_limits_reach_child(self):
        code = "import os; assert len(os.sched_getaffinity(0)) == 1; assert os.environ['OMP_NUM_THREADS'] == '1'; assert os.getpriority(os.PRIO_PROCESS, 0) >= 10"
        result, report = self.run_job([sys.executable, '-c', code], '--cpus', '1')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(report['status'], 'passed')

    def test_failure_is_preserved(self):
        result, report = self.run_job([sys.executable, '-c', 'raise SystemExit(7)'])
        self.assertEqual(result.returncode, 7)
        self.assertEqual(report['status'], 'command_failed')

    def test_timeout_stops_job(self):
        result, report = self.run_job([sys.executable, '-c', 'import time; time.sleep(30)'], '--timeout', '0.4')
        self.assertEqual(result.returncode, 125)
        self.assertEqual(report['reason'], 'command timeout')
        self.assertLess(report['elapsed_seconds'], 5)

    def test_insufficient_ram_prevents_launch(self):
        result, report = self.run_job([sys.executable, '-c', 'raise SystemExit(9)'],
                                      '--min-available-gib', '1000000')
        self.assertEqual(result.returncode, 125)
        self.assertEqual(report['status'], 'blocked_or_stopped')
        self.assertEqual(report['samples'], [])

    def test_term_cleans_up_owned_child_group(self):
        with tempfile.TemporaryDirectory() as directory:
            report = Path(directory) / 'report.json'
            pidfile = Path(directory) / 'child.pid'
            code = "import os, pathlib, time; pathlib.Path(" + repr(str(pidfile)) + ").write_text(str(os.getpid())); time.sleep(30)"
            runner = subprocess.Popen(
                [sys.executable, str(RUNNER), '--report', str(report),
                 '--min-available-gib', '0', '--', sys.executable, '-c', code],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            try:
                deadline = time.monotonic() + 5
                while not pidfile.exists() and time.monotonic() < deadline:
                    time.sleep(0.02)
                self.assertTrue(pidfile.exists())
                child = int(pidfile.read_text())
                runner.send_signal(signal.SIGTERM)
                self.assertEqual(runner.wait(timeout=5), 125)
                with self.assertRaises(ProcessLookupError):
                    os.kill(child, 0)
                self.assertEqual(json.loads(report.read_text())['status'], 'blocked_or_stopped')
            finally:
                if runner.poll() is None:
                    runner.terminate()
                    runner.wait(timeout=5)


if __name__ == '__main__':
    unittest.main()
