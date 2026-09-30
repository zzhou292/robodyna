"""Retention values and fast guard-loop tests with synthetic metrics, no GPU.

Existing subprocess tests separately exercise actual process-group cleanup.
"""

import contextlib
import io
import itertools
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest import mock

from bounded_history import SampleHistory
import run_bounded as runner


def sample(index, rss=None):
    return dict(elapsed_seconds=index, rss_bytes=index if rss is None else rss)


class SampleHistoryTests(unittest.TestCase):
    def test_each_rollover_preserves_first_and_exact_chronological_tail(self):
        history = SampleHistory(7, 2)
        for count in range(1, 101):
            history.append(sample(count - 1))
            expected = list(range(count)) if count <= 7 else [0, 1] + list(range(count - 5, count))
            self.assertEqual([s['elapsed_seconds'] for s in history.samples()], expected)
            self.assertEqual(len(history.first) + len(history.last), min(count, 7))
            self.assertEqual(history.total_samples, count)

    def test_evicted_peak_remains_the_whole_run_maximum(self):
        history = SampleHistory(5, 1)
        for index in range(30):
            history.append(sample(index, 1000000 if index == 2 else index))
        self.assertNotIn(2, [s['elapsed_seconds'] for s in history.samples()])
        self.assertEqual(history.peak_rss_bytes, 1000000)
        self.assertEqual(max(s['rss_bytes'] for s in history.samples()), 29)

    def test_metadata_distinguishes_empty_complete_and_truncated_history(self):
        history = SampleHistory(4, 1)
        for count in range(7):
            if count:
                history.append(sample(count))
            metadata = json.loads(json.dumps(history.metadata()))
            self.assertEqual(metadata, dict(
                policy='first_and_rolling_last_v1', capacity=4,
                first_capacity=1, last_capacity=3, total_samples=count,
                retained_samples=min(count, 4), dropped_samples=max(0, count - 4),
                retained_first_samples=min(count, 1),
                retained_last_samples=min(max(0, count - 1), 3),
                complete_trace=count <= 4))
        self.assertEqual(SampleHistory().peak_rss_bytes, 0)

    def test_default_cap_stays_bounded_over_many_rollovers(self):
        history = SampleHistory()
        for index in range(50000):
            history.append(sample(index))
        values = history.samples()
        self.assertEqual(len(values), 16384)
        self.assertEqual([s['elapsed_seconds'] for s in values[:2048]], list(range(2048)))
        self.assertEqual([s['elapsed_seconds'] for s in values[2048:]], list(range(35664, 50000)))
        self.assertEqual(history.metadata()['dropped_samples'], 33616)

    def test_invalid_capacities_reject_and_tail_only_is_well_defined(self):
        for capacity, first in [(0, 0), (-1, 0), (3, -1), (3, 3), (3, 4),
                                (3.5, 1), (3, True), (True, 0)]:
            with self.subTest(capacity=capacity, first=first):
                with self.assertRaises(ValueError):
                    SampleHistory(capacity, first)
        history = SampleHistory(1, 0)
        history.append(sample(1))
        history.append(sample(2))
        self.assertEqual(history.samples(), [sample(2)])


class BoundedHistoryRunnerTests(unittest.TestCase):
    def run_guard(self, *, breach=None, cooperative=False, preflight=False,
                  diagnostics=False, diagnostic_failure=None, final_failure=None,
                  document_failure=False):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            report_path = root / 'report.json'
            stop_path = root / 'stop.request'
            count = 0

            class Process:
                pid = 12345
                returncode = None

                def poll(self):
                    if count >= 14:
                        self.returncode = 0
                    return self.returncode

            process = Process()

            session = mock.Mock()
            session.record = dict(policy='owned_session_v1')
            session.exited.side_effect = lambda: process.poll() is not None

            diagnostic_events = []

            def stop_session():
                diagnostic_events.append('stop')
                if process.returncode is None:
                    process.returncode = -15

            session.stop.side_effect = stop_session

            gpu_processes = mock.Mock()

            def capture(_session, trigger, _gpu, _poll_time):
                self.assertIs(_session, session)
                diagnostic_events.append(trigger)
                failure = final_failure if trigger == 'before_gpu_growth_stop' else diagnostic_failure
                if failure is not None:
                    raise failure

            gpu_processes.sample.side_effect = capture
            gpu_processes.document.side_effect = (
                RuntimeError('diagnostic serialization failed') if document_failure
                else lambda: dict(events=diagnostic_events))

            def memory():
                bad = preflight or (breach == 'RAM' and count >= 11)
                return dict(MemAvailable=(0 if bad else 16 * runner.GIB),
                            MemFree=8 * runner.GIB)

            def usage():
                nonlocal count
                count += 1
                rss = 16 * runner.MIB if count == 3 else runner.MIB
                if breach == 'RSS' and count >= 12:
                    rss = 2 * runner.GIB
                return dict(rss_bytes=rss, live_cpu_ticks=count, threads=1)

            def gpu(_):
                used, free = 1, 24
                if (cooperative and count == 3) or (breach == 'growth' and count >= 11):
                    used = 8
                if breach == 'free' and count >= 11:
                    free = 4
                return dict(total_bytes=32 * runner.GIB, used_bytes=used * runner.GIB,
                            free_bytes=free * runner.GIB, utilization_percent=0)

            args = ['run_bounded.py', '--report', str(report_path), '--cpus', '1',
                    '--min-available-gib', '1', '--max-rss-gib', '1',
                    '--timeout', '10.5' if breach == 'timeout' else '1000']
            if cooperative or diagnostics or breach in ('free', 'growth'):
                args += ['--gpu', '0', '--max-gpu-growth-gib', '6']
            if diagnostics:
                args += ['--gpu-process-diagnostics']
            if cooperative:
                args += ['--cooperative-stop-file', str(stop_path),
                         '--cooperative-stop-grace-seconds', '100']
            args += ['--', 'synthetic-child']
            with contextlib.ExitStack() as stack:
                stack.enter_context(mock.patch.object(sys, 'argv', args))
                stack.enter_context(mock.patch.object(runner, 'SampleHistory',
                                                      side_effect=lambda: SampleHistory(6, 2)))
                stack.enter_context(mock.patch.object(runner, 'memory_info', side_effect=memory))
                session.usage.side_effect = usage
                stack.enter_context(mock.patch.object(runner, 'OwnedSession', return_value=session))
                stack.enter_context(mock.patch.object(runner, 'gpu_info', side_effect=gpu))
                stack.enter_context(mock.patch.object(runner, 'GpuProcessDiagnostics',
                                                      return_value=gpu_processes))
                spawn = stack.enter_context(mock.patch.object(runner.subprocess, 'Popen', return_value=process))
                stopped = session.stop
                stack.enter_context(mock.patch.object(runner.time, 'monotonic', side_effect=itertools.count()))
                stack.enter_context(mock.patch.object(runner.time, 'sleep'))
                stack.enter_context(contextlib.redirect_stdout(io.StringIO()))
                stack.enter_context(contextlib.redirect_stderr(io.StringIO()))
                result = runner.main()
            report = json.loads(report_path.read_text())
            return result, report, count, spawn.call_count, stopped.call_count, stop_path.exists()

    def test_complete_command_report_keeps_evicted_peak_and_explicit_metadata(self):
        result, report, count, _, stopped, _ = self.run_guard()
        self.assertEqual(result, 0)
        self.assertEqual(report['status'], 'passed')
        self.assertEqual(count, 14)
        self.assertEqual(stopped, 1)
        self.assertEqual([s['live_cpu_ticks'] for s in report['samples']], [1, 2, 11, 12, 13, 14])
        self.assertEqual(report['peak_sampled_rss_bytes'], 16 * runner.MIB)
        self.assertTrue(all(s['rss_bytes'] == runner.MIB for s in report['samples']))
        self.assertEqual(report['sample_history']['total_samples'], 14)
        self.assertEqual(report['sample_history']['retained_samples'], 6)
        self.assertEqual(report['sample_history']['dropped_samples'], 8)
        self.assertFalse(report['sample_history']['complete_trace'])

    def test_every_limit_still_enforces_after_history_rollover(self):
        reasons = dict(RAM='available RAM fell below reserve', RSS='job RSS exceeded budget',
                       free='free GPU memory fell below reserve',
                       growth='total GPU memory growth exceeded job allowance',
                       timeout='command timeout')
        for breach, reason in reasons.items():
            with self.subTest(breach=breach):
                result, report, count, _, stopped, _ = self.run_guard(breach=breach)
                self.assertEqual(result, 125)
                self.assertEqual(report['reason'], reason)
                self.assertEqual(stopped, 1)
                self.assertGreater(count, 6)
                self.assertEqual(report['sample_history']['total_samples'], count)
                self.assertEqual(report['sample_history']['dropped_samples'], count - 6)
                self.assertEqual(report['samples'][-1]['live_cpu_ticks'], count)

    def test_cooperative_trigger_is_preserved_after_regular_sample_eviction(self):
        result, report, count, _, stopped, requested = self.run_guard(cooperative=True)
        self.assertEqual(result, 0)
        self.assertEqual(report['status'], 'cooperatively_stopped')
        self.assertTrue(requested)
        self.assertEqual(stopped, 1)
        self.assertEqual(report['sample_history']['total_samples'], count)
        trigger = report['cooperative_stop']['trigger_sample']
        self.assertEqual(trigger['live_cpu_ticks'], 3)
        self.assertEqual(trigger['gpu']['used_bytes'], 8 * runner.GIB)
        self.assertNotIn(3, [s['live_cpu_ticks'] for s in report['samples']])
        self.assertEqual(report['cooperative_stop']['outcome'], 'cooperative_exit')

    def test_gpu_diagnostics_require_gpu_before_child_launch(self):
        args = ['run_bounded.py', '--report', '/unused/report.json',
                '--gpu-process-diagnostics', '--', 'synthetic-child']
        with mock.patch.object(sys, 'argv', args), \
                mock.patch.object(runner.subprocess, 'Popen') as spawn, \
                contextlib.redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit) as error:
                runner.main()
        self.assertEqual(error.exception.code, 2)
        spawn.assert_not_called()

    def test_optional_gpu_diagnostics_default_off_and_successful_poll_scope(self):
        _, plain, *_ = self.run_guard()
        self.assertNotIn('gpu_process_diagnostics', plain)
        result, observed, _, _, stopped, _ = self.run_guard(diagnostics=True)
        self.assertEqual(result, 0)
        self.assertEqual(stopped, 1)
        events = observed['gpu_process_diagnostics']['events']
        self.assertIn('poll', events)
        self.assertNotIn('before_gpu_growth_stop', events)
        self.assertEqual(events[-1], 'stop')

    def test_gpu_diagnostics_preserve_every_original_hard_guard(self):
        for breach in ('RAM', 'RSS', 'free', 'growth', 'timeout'):
            with self.subTest(breach=breach):
                _, expected, *_ = self.run_guard(breach=breach)
                result, actual, _, _, stopped, _ = self.run_guard(breach=breach, diagnostics=True)
                self.assertEqual(result, 125)
                self.assertEqual(actual['reason'], expected['reason'])
                self.assertEqual(stopped, 1)
                events = actual['gpu_process_diagnostics']['events']
                if breach == 'growth':
                    self.assertEqual(events[-2:], ['before_gpu_growth_stop', 'stop'])
                else:
                    self.assertNotIn('before_gpu_growth_stop', events)

    def test_gpu_diagnostic_poll_and_document_errors_are_informational(self):
        result, report, _, _, stopped, _ = self.run_guard(
            diagnostics=True, diagnostic_failure=RuntimeError('optional query failed'))
        self.assertEqual(result, 0)
        self.assertEqual(stopped, 1)
        self.assertEqual(report['gpu_process_diagnostic_error'], 'optional query failed')
        result, report, _, _, stopped, _ = self.run_guard(diagnostics=True, document_failure=True)
        self.assertEqual(result, 0)
        self.assertEqual(stopped, 1)
        self.assertEqual(report['gpu_process_diagnostics']['status'], 'diagnostic_error')

    def test_final_diagnostic_failures_and_interruptions_cannot_skip_cleanup(self):
        for failure in (RuntimeError('query error'), KeyboardInterrupt(),
                        runner.GuardInterrupted('runner interrupted by signal 15')):
            with self.subTest(failure=type(failure).__name__):
                result, report, _, _, stopped, _ = self.run_guard(
                    diagnostics=True, breach='growth', final_failure=failure)
                self.assertEqual(result, 125)
                self.assertEqual(report['reason'], runner.GPU_GROWTH_REASON)
                self.assertEqual(stopped, 1)
                self.assertNotIn('cleanup_error', report)
                self.assertEqual(report['gpu_process_diagnostics']['events'][-1], 'stop')

    def test_interrupt_during_optional_poll_preserves_signal_reason(self):
        result, report, _, _, stopped, _ = self.run_guard(
            diagnostics=True,
            diagnostic_failure=runner.GuardInterrupted('runner interrupted by signal 15'))
        self.assertEqual(result, 125)
        self.assertEqual(report['reason'], 'runner interrupted by signal 15')
        self.assertEqual(stopped, 1)
        self.assertNotIn('cleanup_error', report)

    def test_preflight_report_has_zero_counters_and_never_launches(self):
        result, report, count, spawned, stopped, _ = self.run_guard(preflight=True)
        self.assertEqual(result, 125)
        self.assertEqual((count, spawned, stopped), (0, 0, 0))
        self.assertEqual(report['samples'], [])
        self.assertEqual(report['peak_sampled_rss_bytes'], 0)
        self.assertEqual(report['sample_history']['total_samples'], 0)
        self.assertEqual(report['sample_history']['dropped_samples'], 0)
        self.assertTrue(report['sample_history']['complete_trace'])


if __name__ == '__main__':
    unittest.main()
