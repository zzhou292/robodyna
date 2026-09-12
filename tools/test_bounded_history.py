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
    def run_guard(self, *, breach=None, cooperative=False, preflight=False):
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

            def stop_group(child):
                if child.returncode is None:
                    child.returncode = -15

            def memory():
                bad = preflight or (breach == 'RAM' and count >= 11)
                return dict(MemAvailable=(0 if bad else 16 * runner.GIB),
                            MemFree=8 * runner.GIB)

            def usage(_):
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
            if cooperative or breach in ('free', 'growth'):
                args += ['--gpu', '0', '--max-gpu-growth-gib', '6']
            if cooperative:
                args += ['--cooperative-stop-file', str(stop_path),
                         '--cooperative-stop-grace-seconds', '100']
            args += ['--', 'synthetic-child']
            with contextlib.ExitStack() as stack:
                stack.enter_context(mock.patch.object(sys, 'argv', args))
                stack.enter_context(mock.patch.object(runner, 'SampleHistory',
                                                      side_effect=lambda: SampleHistory(6, 2)))
                stack.enter_context(mock.patch.object(runner, 'memory_info', side_effect=memory))
                stack.enter_context(mock.patch.object(runner, 'group_usage', side_effect=usage))
                stack.enter_context(mock.patch.object(runner, 'gpu_info', side_effect=gpu))
                spawn = stack.enter_context(mock.patch.object(runner.subprocess, 'Popen', return_value=process))
                stopped = stack.enter_context(mock.patch.object(runner, 'stop_group', side_effect=stop_group))
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
