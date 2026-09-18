"""Host-only GPU telemetry contracts; no real process query or GPU execution."""

from contextlib import ExitStack
import unittest
from unittest import mock

import bounded_gpu as gpu
import bounded_gpu_query as query
from bounded_session import ProcessRow


def process(pid, *, sid=100, start=7, state='S'):
    return ProcessRow(pid, 1, pid, sid, start, state, 2, 3, 1)


def inventory(*rows):
    return {row.pid: row for row in rows}


class ComputeInventoryTests(unittest.TestCase):
    def test_empty_inventory_and_unavailable_memory_remain_distinct(self):
        self.assertEqual(gpu.parse_compute_apps(b'\n \n'), ([], 0))
        tokens = ['N/A', '[N/A]', 'Not Supported', '[Not Supported]', '-', '0', '17']
        payload = ''.join(f'{i + 1}, {value}\n' for i, value in enumerate(tokens)).encode()
        rows, count = gpu.parse_compute_apps(payload)
        self.assertEqual(count, len(tokens))
        self.assertEqual([row['used_bytes'] for row in rows],
                         [None] * 5 + [0, 17 * gpu.MIB])

    def test_malformed_and_duplicate_rows_reject(self):
        payloads = [b'0, 1', b'-1, 1', b'2147483648, 1', b'1', b'1, 1, 2',
                    b'1, -1', b'1, 1.5', b'1, nan', b'1, null', b'1, ',
                    b'1, 1\n1, 2', b'1, 1\n01, 2', b'\xff, 1',
                    b'1, 18446744073709551615']
        for payload in payloads:
            with self.subTest(payload=payload):
                with self.assertRaises(ValueError):
                    gpu.parse_compute_apps(payload)

    def test_unsigned_byte_conversion_checks_exact_overflow_boundary(self):
        maximum = (2**64 - 1) // gpu.MIB
        rows, count = gpu.parse_compute_apps(f'2147483647, {maximum}'.encode())
        self.assertEqual(count, 1)
        self.assertEqual(rows[0]['used_bytes'], maximum * gpu.MIB)
        with self.assertRaises(ValueError):
            gpu.parse_compute_apps(f'1, {maximum + 1}'.encode())

    def test_row_cap_retains_count_and_validates_omitted_tail(self):
        count = gpu.MAX_PROCESS_ROWS + 5
        payload = ''.join(f'{i + 1}, {i}\n' for i in range(count)).encode()
        rows, actual = gpu.parse_compute_apps(payload)
        self.assertEqual(actual, count)
        self.assertEqual(len(rows), gpu.MAX_PROCESS_ROWS)
        self.assertEqual(rows[-1]['pid'], gpu.MAX_PROCESS_ROWS)
        for tail in (b'1, 99\n', b'9999, malformed\n'):
            with self.subTest(tail=tail):
                with self.assertRaises(ValueError):
                    gpu.parse_compute_apps(payload + tail)

    def test_output_byte_cap_rejects_before_decoding(self):
        self.assertEqual(gpu.parse_compute_apps(b' ' * gpu.OUTPUT_CAP), ([], 0))
        with self.assertRaisesRegex(ValueError, 'byte cap'):
            gpu.parse_compute_apps(b' ' * (gpu.OUTPUT_CAP + 1))


class ComputeMembershipTests(unittest.TestCase):
    def classify(self, values, first=(), last=(), before=(), after=()):
        return gpu.classify_processes(values, inventory(*first), inventory(*last),
                                      inventory(*before), inventory(*after).get, 100)

    def test_authenticated_owned_other_and_unknown_subtotals(self):
        own = process(101)
        external = process(201, sid=200)
        values = [dict(pid=101, used_bytes=3 * gpu.MIB),
                  dict(pid=201, used_bytes=None), dict(pid=301, used_bytes=0)]
        rows, totals = self.classify(values, [own], [own], [own, external],
                                    [own, external])
        self.assertEqual([row['membership'] for row in rows], ['owned', 'other', 'unknown'])
        self.assertEqual([row['process_start_ticks'] for row in rows], [7, 7, None])
        self.assertEqual(totals, {
            'owned': dict(processes=1, known_used_bytes=3 * gpu.MIB, unknown_memory_processes=0),
            'other': dict(processes=1, known_used_bytes=0, unknown_memory_processes=1),
            'unknown': dict(processes=1, known_used_bytes=0, unknown_memory_processes=0)})
        self.assertNotIn('membership', values[0])

    def test_pid_reuse_exit_session_change_and_partial_membership_are_unknown(self):
        original = process(101)
        variants = [dict(last=[process(101, start=8)], after=[process(101, start=8)]),
                    dict(last=[original], after=[process(101, start=8)]),
                    dict(last=[original], after=[]),
                    dict(last=[original], after=[process(101, state='Z')]),
                    dict(last=[original], after=[process(101, sid=200)]),
                    dict(last=[], after=[original])]
        for variant in variants:
            with self.subTest(variant=variant):
                rows, totals = self.classify([dict(pid=101, used_bytes=None)],
                                            first=[original], before=[original], **variant)
                self.assertEqual(rows[0]['membership'], 'unknown')
                self.assertIsNone(rows[0]['process_start_ticks'])
                self.assertEqual(totals['unknown']['unknown_memory_processes'], 1)

    def test_capped_membership_never_makes_same_session_process_external(self):
        missed = process(999)
        external = process(998, sid=200)
        rows, _ = self.classify([dict(pid=999, used_bytes=11), dict(pid=998, used_bytes=12)],
                                before=[missed, external], after=[missed, external])
        self.assertEqual([row['membership'] for row in rows], ['unknown', 'other'])
        rows, _ = self.classify([dict(pid=998, used_bytes=12)],
                                before=[external], after=[process(998, start=8, sid=200)])
        self.assertEqual(rows[0]['membership'], 'unknown')

    def test_read_errors_retain_unattributed_memory_in_unknown_subtotal(self):
        row = process(101)
        for error in (PermissionError(), ValueError(), IndexError()):
            with self.subTest(error=type(error).__name__):
                result, totals = gpu.classify_processes(
                    [dict(pid=101, used_bytes=19)], inventory(row), inventory(row),
                    inventory(row), mock.Mock(side_effect=error), 100)
                self.assertEqual(result[0]['membership'], 'unknown')
                self.assertEqual(totals['unknown']['known_used_bytes'], 19)


class ComputeQueryTests(unittest.TestCase):
    def query_with(self, chunks, *, code=0, ready=True, running=False, cap=None):
        child = mock.Mock()
        child.stdout.fileno.return_value = 91
        child.wait.return_value = code
        child.poll.return_value = None if running else code
        selector = mock.MagicMock()
        selector.__enter__.return_value = selector
        selector.select.return_value = [(None, None)] if ready else []
        with ExitStack() as stack:
            launch = stack.enter_context(mock.patch.object(query.subprocess, 'Popen', return_value=child))
            stack.enter_context(mock.patch.object(query.selectors, 'DefaultSelector', return_value=selector))
            reader = stack.enter_context(mock.patch.object(query.os, 'read', side_effect=chunks))
            stack.enter_context(mock.patch.object(query.time, 'monotonic', return_value=10.0))
            if cap is not None:
                stack.enter_context(mock.patch.object(query, 'OUTPUT_CAP', cap))
            result = query.query_compute_apps(2)
        self.assertEqual(launch.call_args.args[0], [
            'nvidia-smi', '-i', '2', '--query-compute-apps=pid,used_gpu_memory',
            '--format=csv,noheader,nounits'])
        self.assertIs(launch.call_args.kwargs['stderr'], query.subprocess.DEVNULL)
        child.stdout.close.assert_called_once_with()
        return result, child, reader

    def test_query_success_drains_split_output_and_reaps(self):
        result, child, reader = self.query_with([b'101, ', b'5\n', b''])
        self.assertEqual(result, query.QueryResult('ok', b'101, 5\n', 0))
        self.assertEqual(reader.call_count, 3)
        child.kill.assert_not_called()
        self.assertTrue(child.wait.called)
        self.assertTrue(all(call.kwargs['timeout'] <= 1 for call in child.wait.call_args_list))

    def test_query_output_limit_discards_partial_bytes_and_reaps_live_child(self):
        result, child, reader = self.query_with([b'123456789'], running=True, cap=8)
        self.assertEqual(result.status, 'query_output_limit')
        self.assertEqual(result.data, b'')
        reader.assert_called_once_with(91, 9)
        child.kill.assert_called_once_with()
        child.wait.assert_called_once_with(timeout=1)

    def test_query_timeout_and_nonzero_exit_have_no_complete_inventory(self):
        result, child, reader = self.query_with([], ready=False, running=True)
        self.assertEqual(result.status, 'query_timeout')
        self.assertEqual(result.data, b'')
        reader.assert_not_called()
        child.kill.assert_called_once_with()
        child.wait.assert_called_once_with(timeout=1)
        result, child, _ = self.query_with([b'101, 5\n', b''], code=9)
        self.assertEqual(result, query.QueryResult('query_failed', returncode=9))
        child.kill.assert_not_called()

    def test_unavailable_query_is_explicit_without_launching_any_real_process(self):
        with mock.patch.object(query.subprocess, 'Popen', side_effect=FileNotFoundError()):
            result = query.query_compute_apps(0)
        self.assertEqual(result.status, 'query_unavailable')
        self.assertEqual(result.data, b'')


class ComputeSnapshotTests(unittest.TestCase):
    def test_capture_uses_bounded_two_sided_membership_and_keeps_cap_uncertainty(self):
        owner = process(101)
        external = process(201, sid=200)
        lost_after_cap = process(301)
        session = mock.Mock(record={'session_id': 100})
        session.process_snapshot.side_effect = [
            ([owner, external, lost_after_cap], True), ([owner, external], False)]
        observer = gpu.GpuProcessDiagnostics(0, 0)
        with mock.patch.object(gpu, 'query_compute_apps', return_value=query.QueryResult(
                'ok', b'101, 3\n201, N/A\n301, 7\n', 0)) as read:
            result = observer._capture(session)
        read.assert_called_once_with(0)
        self.assertEqual(session.process_snapshot.call_args_list,
                         [mock.call(gpu.MAX_IDENTITY_ROWS)] * 2)
        self.assertTrue(result['membership_before_complete'])
        self.assertFalse(result['membership_after_complete'])
        self.assertTrue(result['rows_complete'])
        self.assertEqual([row['membership'] for row in result['rows']],
                         ['owned', 'other', 'unknown'])
        self.assertEqual(result['totals']['unknown']['known_used_bytes'], 7 * gpu.MIB)
        session.members.assert_not_called()

    def test_capture_reports_omitted_rows_and_subtotals_only_retained_inventory(self):
        count = gpu.MAX_PROCESS_ROWS + 2
        roster = [process(101 + index) for index in range(count)]
        session = mock.Mock(record={'session_id': 100})
        session.process_snapshot.return_value = (roster, True)
        payload = ''.join(f'{row.pid}, 1\n' for row in roster).encode()
        observer = gpu.GpuProcessDiagnostics(0, 0)
        with mock.patch.object(gpu, 'query_compute_apps',
                               return_value=query.QueryResult('ok', payload, 0)):
            result = observer._capture(session)
        self.assertEqual(result['row_count'], count)
        self.assertEqual(result['retained_rows'], gpu.MAX_PROCESS_ROWS)
        self.assertEqual(result['omitted_rows'], 2)
        self.assertFalse(result['rows_complete'])
        self.assertEqual(result['totals']['owned']['processes'], gpu.MAX_PROCESS_ROWS)
        self.assertEqual(result['totals']['owned']['known_used_bytes'],
                         gpu.MAX_PROCESS_ROWS * gpu.MIB)
        self.assertIn('retained', result['totals_scope'])

    def test_failed_query_has_unknown_inventory_not_zero_owned_memory(self):
        session = mock.Mock(record={'session_id': 100})
        session.process_snapshot.return_value = ([process(101)], True)
        observer = gpu.GpuProcessDiagnostics(0, 0)
        with mock.patch.object(gpu, 'query_compute_apps',
                               return_value=query.QueryResult('query_timeout')):
            result = observer._capture(session)
        self.assertEqual(result['status'], 'query_timeout')
        self.assertIsNone(result['totals'])
        self.assertIsNone(result['row_count'])
        self.assertFalse(result['rows_complete'])
        session.process_snapshot.assert_called_once_with(gpu.MAX_IDENTITY_ROWS)

    def test_bounded_tail_preserves_initial_and_stop_snapshot(self):
        observer = gpu.GpuProcessDiagnostics(0, 10)
        count = gpu.MAX_SNAPSHOTS + 3
        data = dict(status='ok', rows=[], row_count=0, rows_complete=True, totals={})
        with mock.patch.object(observer, '_capture', return_value=data), \
                mock.patch.object(gpu.time, 'monotonic', return_value=11):
            for index in range(count):
                trigger = 'before_gpu_growth_stop' if index == count - 1 else 'periodic'
                observer.sample(None, trigger, dict(used_bytes=17), 0.75)
        result = observer.document()
        self.assertEqual(result['total_snapshots'], count)
        self.assertEqual(result['tail_snapshots'], gpu.MAX_SNAPSHOTS)
        self.assertEqual(result['tail_evicted_snapshots'], 3)
        self.assertEqual(result['initial_snapshot']['sequence'], 1)
        self.assertEqual(result['snapshots'][0]['sequence'], 4)
        self.assertEqual(result['snapshots'][-1]['sequence'], count)
        self.assertEqual(result['before_gpu_growth_stop'], result['snapshots'][-1])
        self.assertEqual(result['snapshots'][-1]['whole_device_used_bytes_at_guard_poll'], 17)
        self.assertEqual(result['snapshots'][-1]['gpu_poll_elapsed_seconds'], 0.75)
        self.assertIn('no leak', result['scope'])

    def test_diagnostic_error_is_bounded_and_never_escapes_to_guard(self):
        observer = gpu.GpuProcessDiagnostics(0, 0)
        with mock.patch.object(observer, '_capture', side_effect=RuntimeError('x' * 1000)):
            observer.sample(None, 'before_gpu_growth_stop', dict(used_bytes=23), 1.0)
        result = observer.document()['before_gpu_growth_stop']
        self.assertEqual(result['status'], 'diagnostic_error')
        self.assertEqual(result['error_type'], 'RuntimeError')
        self.assertEqual(len(result['error']), 256)
        self.assertIsNone(result['totals'])
        self.assertIsNone(result['row_count'])
        self.assertFalse(result['rows_complete'])
        self.assertEqual(result['rows'], [])


if __name__ == '__main__':
    unittest.main()
