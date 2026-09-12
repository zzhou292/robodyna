"""Real small children with mocked workstation metrics; never queries a GPU."""

import contextlib
import io
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest import mock

import run_bounded as runner
from bounded_stop import GPU_GROWTH_REASON


def gpu(used, free=24):
    return dict(total_bytes=32 * runner.GIB, used_bytes=used * runner.GIB,
                free_bytes=free * runner.GIB, utilization_percent=0)


class CooperativeStopTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.root = Path(self.directory.name)
        self.stop = self.root / 'stop.request'
        self.report = self.root / 'report.json'
        self.prefix = self.root / 'prefix.json'

    def cooperative_child(self, exit_code=0):
        return ("import pathlib,time; "
                f"stop=pathlib.Path({str(self.stop)!r}); "
                f"prefix=pathlib.Path({str(self.prefix)!r})\n"
                "while not stop.exists(): time.sleep(.005)\n"
                "prefix.write_text('accepted prefix')\n"
                f"raise SystemExit({exit_code})")

    def run_guard(self, code, *, enabled=True, grace=.8, timeout=5,
                  memory=None, usage=None, gpu_samples=None, spawn=None):
        arguments = ['run_bounded.py', '--report', str(self.report),
                     '--cpus', '1', '--gpu', '0', '--min-available-gib', '1',
                     '--max-rss-gib', '1', '--min-gpu-free-gib', '8',
                     '--max-gpu-growth-gib', '6', '--timeout', str(timeout)]
        if enabled:
            arguments += ['--cooperative-stop-file', str(self.stop),
                          '--cooperative-stop-grace-seconds', str(grace)]
        arguments += ['--', sys.executable, '-c', code]
        good_memory = dict(MemAvailable=16 * runner.GIB, MemFree=8 * runner.GIB)
        good_usage = dict(rss_bytes=4 * runner.MIB, live_cpu_ticks=0, threads=1)
        output = io.StringIO()
        with contextlib.ExitStack() as stack:
            stack.enter_context(mock.patch.object(sys, 'argv', arguments))
            stack.enter_context(mock.patch.object(runner, 'memory_info',
                                                  side_effect=memory or (lambda: good_memory)))
            stack.enter_context(mock.patch.object(runner, 'group_usage',
                                                  side_effect=usage or (lambda _: good_usage)))
            metrics = stack.enter_context(mock.patch.object(runner, 'gpu_info',
                                                             side_effect=gpu_samples or [gpu(1), gpu(8)]))
            if spawn is not None:
                stack.enter_context(mock.patch.object(runner.subprocess, 'Popen', side_effect=spawn))
            stack.enter_context(contextlib.redirect_stdout(output))
            stack.enter_context(contextlib.redirect_stderr(output))
            code = runner.main()
        return code, json.loads(self.report.read_text()), metrics.call_count

    def test_default_growth_breach_still_kills_without_request(self):
        code, report, calls = self.run_guard('import time; time.sleep(30)', enabled=False)
        self.assertEqual(code, 125)
        self.assertEqual(report['reason'], GPU_GROWTH_REASON)
        self.assertEqual(report['status'], 'blocked_or_stopped')
        self.assertNotIn('cooperative_stop', report)
        self.assertFalse(self.stop.exists())
        self.assertEqual(calls, 2)

    def test_growth_requests_once_and_child_publishes_prefix_before_zero_exit(self):
        code, report, _ = self.run_guard(self.cooperative_child())
        self.assertEqual(code, 0)
        self.assertEqual(self.prefix.read_text(), 'accepted prefix')
        self.assertEqual(report['status'], 'cooperatively_stopped')
        self.assertEqual(report['reason'], GPU_GROWTH_REASON)
        event = report['cooperative_stop']
        self.assertEqual(event['outcome'], 'cooperative_exit')
        self.assertEqual(event['child_exit_code'], 0)
        self.assertTrue(event['request_created'])
        self.assertTrue(event['request_completed'])
        self.assertEqual(event['trigger_sample']['gpu']['free_bytes'], 24 * runner.GIB)
        self.assertEqual(event['grace_deadline_elapsed_seconds'], event['request_elapsed_seconds'] + .8)
        self.assertLess(event['completion_elapsed_seconds'], event['grace_deadline_elapsed_seconds'])
        self.assertEqual(json.loads(self.stop.read_text())['reason'], GPU_GROWTH_REASON)

    def test_nonzero_cooperative_child_exit_is_not_a_successful_prefix_claim(self):
        code, report, _ = self.run_guard(self.cooperative_child(7))
        self.assertEqual(code, 7)
        self.assertEqual(report['status'], 'command_failed')
        self.assertEqual(report['cooperative_stop']['outcome'], 'command_failed')

    def test_ignored_request_expires_and_kills_even_after_growth_recovers(self):
        code, report, calls = self.run_guard('import time; time.sleep(30)', grace=2.1,
                                             gpu_samples=[gpu(1), gpu(8), gpu(1)])
        self.assertEqual(code, 125)
        self.assertEqual(calls, 3)
        self.assertEqual(report['reason'], 'cooperative stop grace expired')
        event = report['cooperative_stop']
        self.assertEqual(event['outcome'], 'forced_stop')
        self.assertGreaterEqual(event['termination_requested_elapsed_seconds'], event['grace_deadline_elapsed_seconds'])
        self.assertGreaterEqual(event['termination_completed_elapsed_seconds'], event['termination_requested_elapsed_seconds'])

    def test_hard_ram_or_rss_breach_during_grace_still_kills(self):
        for kind in ['RAM', 'RSS']:
            with self.subTest(kind=kind):
                self.stop.unlink(missing_ok=True)
                calls = 0

                def memory():
                    nonlocal calls
                    calls += 1
                    return dict(MemAvailable=(0 if kind == 'RAM' and calls >= 3 else 16 * runner.GIB),
                                MemFree=8 * runner.GIB)

                def usage(_):
                    return dict(rss_bytes=(2 * runner.GIB if kind == 'RSS' and calls >= 3 else runner.MIB),
                                live_cpu_ticks=0, threads=1)

                code, report, _ = self.run_guard('import time; time.sleep(30)', memory=memory, usage=usage)
                self.assertEqual(code, 125)
                expected = 'available RAM fell below reserve' if kind == 'RAM' else 'job RSS exceeded budget'
                self.assertEqual(report['reason'], expected)
                self.assertEqual(report['cooperative_stop']['outcome'], 'forced_stop')
                self.assertTrue(self.stop.exists())

    def test_hard_gpu_free_reserve_breach_during_grace_still_kills(self):
        code, report, _ = self.run_guard('import time; time.sleep(30)', grace=4,
                                         gpu_samples=[gpu(1), gpu(8), gpu(26, free=6)])
        self.assertEqual(code, 125)
        self.assertEqual(report['reason'], 'free GPU memory fell below reserve')
        self.assertEqual(report['cooperative_stop']['outcome'], 'forced_stop')

    def test_command_timeout_remains_hard_during_grace(self):
        code, report, _ = self.run_guard('import time; time.sleep(30)', timeout=.1)
        self.assertEqual(code, 125)
        self.assertEqual(report['reason'], 'command timeout')
        self.assertEqual(report['cooperative_stop']['outcome'], 'forced_stop')

    def test_no_growth_preserves_normal_exit_and_creates_no_stop_file(self):
        code, report, _ = self.run_guard('raise SystemExit(0)', gpu_samples=[gpu(1), gpu(1)])
        self.assertEqual(code, 0)
        self.assertEqual(report['status'], 'passed')
        self.assertEqual(report['cooperative_stop']['outcome'], 'not_requested')
        self.assertFalse(self.stop.exists())

    def test_hard_breach_at_first_sample_does_not_request(self):
        code, report, _ = self.run_guard('import time; time.sleep(30)', gpu_samples=[gpu(1), gpu(26, free=6)])
        self.assertEqual(code, 125)
        self.assertEqual(report['reason'], 'free GPU memory fell below reserve')
        self.assertFalse(self.stop.exists())

    def test_request_collision_after_preflight_preserves_file_and_kills(self):
        original_spawn = subprocess.Popen

        def spawn(*args, **kwargs):
            self.stop.write_text('foreign existing content')
            return original_spawn(*args, **kwargs)

        code, report, _ = self.run_guard('import time; time.sleep(30)', spawn=spawn)
        self.assertEqual(code, 125)
        self.assertEqual(self.stop.read_text(), 'foreign existing content')
        self.assertEqual(report['cooperative_stop']['outcome'], 'request_failed')
        self.assertFalse(report['cooperative_stop']['request_created'])

    def test_existing_file_symlink_and_report_alias_reject_before_launch(self):
        for kind in ['file', 'symlink', 'report']:
            with self.subTest(kind=kind):
                self.stop.unlink(missing_ok=True)
                if kind == 'file':
                    self.stop.write_text('keep')
                elif kind == 'symlink':
                    self.stop.symlink_to(self.root / 'absent-target')
                else:
                    self.stop = self.report
                code, report, calls = self.run_guard('raise SystemExit(9)')
                self.assertEqual(code, 125)
                self.assertEqual(calls, 0)
                self.assertEqual(report['samples'], [])
                self.assertEqual(report['cooperative_stop']['outcome'], 'not_requested')

    def test_invalid_grace_or_missing_gpu_is_rejected_by_cli(self):
        for extra in [[], ['--cooperative-stop-grace-seconds', '0'],
                      ['--cooperative-stop-grace-seconds', 'nan'],
                      ['--cooperative-stop-grace-seconds', 'inf']]:
            arguments = [sys.executable, str(Path(runner.__file__)), '--report', str(self.report),
                         '--cooperative-stop-file', str(self.stop)]
            if extra:
                arguments += ['--gpu', '0'] + extra
            result = subprocess.run(arguments + ['--', sys.executable, '-c', 'raise SystemExit(9)'],
                                    capture_output=True, text=True, timeout=5)
            self.assertEqual(result.returncode, 2)
            self.assertFalse(self.stop.exists())


if __name__ == '__main__':
    unittest.main()
