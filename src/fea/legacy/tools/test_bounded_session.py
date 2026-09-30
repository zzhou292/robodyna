"""Bounded fake /proc controls for session identity and cleanup races."""

import os
from pathlib import Path
import signal
import tempfile
import unittest
from unittest import mock

import bounded_session as scope


def stat_line(pid, *, ppid=None, pgid=None, sid=100, start=7,
              state='S', rss=2, ticks=3, threads=1):
    fields = ['0'] * 22
    fields[0] = state
    fields[1] = str(os.getpid() if ppid is None else ppid)
    fields[2] = str(pid if pgid is None else pgid)
    fields[3] = str(sid)
    fields[11] = str(ticks)
    fields[12] = '2'
    fields[17] = str(threads)
    fields[19] = str(start)
    fields[21] = str(rss)
    return f'{pid} (comm with ) spaces) ' + ' '.join(fields) + '\n'


class SessionValuesTests(unittest.TestCase):
    def setUp(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.root = Path(directory.name)
        self.write(100)
        self.process = mock.Mock(pid=100)
        self.session = scope.OwnedSession(self.process, self.root)

    def write(self, pid, **values):
        path = self.root / str(pid)
        path.mkdir(exist_ok=True)
        (path / 'stat').write_text(stat_line(pid, **values))

    def test_usage_includes_all_session_groups_and_excludes_other_sessions(self):
        self.write(101, ppid=100, pgid=101, rss=17, ticks=11, threads=4)
        self.write(102, ppid=101, pgid=101, rss=31, ticks=19, threads=2)
        self.write(103, ppid=100, pgid=103, sid=103, rss=9999)
        self.write(104, ppid=1, sid=104, rss=9999)
        (self.root / '105').mkdir()  # Vanished process, no stat.
        (self.root / 'junk').mkdir()
        self.write(106)
        (self.root / '106' / 'stat').write_text('invalid')
        self.assertEqual(self.session.usage(), dict(
            rss_bytes=50 * os.sysconf('SC_PAGE_SIZE'),
            live_cpu_ticks=39, threads=7))
        self.assertEqual({row.pgid for row in self.session.members()}, {100, 101})

    def test_diagnostic_snapshot_caps_reads_and_preserves_normal_members(self):
        self.write(101, ppid=100, sid=100)
        self.write(102, ppid=1, sid=102)
        rows = [scope.read_process(pid, self.root) for pid in (100, 101, 102)]
        visited = []

        def inventory(_root):
            for row in rows:
                visited.append(row.pid)
                yield row

        with mock.patch.object(scope, 'process_rows', side_effect=inventory), \
                mock.patch.object(self.session, '_check_leader', wraps=self.session._check_leader) as check:
            snapshot, complete = self.session.process_snapshot(1)
        self.assertEqual([row.pid for row in snapshot], [100])
        self.assertFalse(complete)
        self.assertEqual(visited, [100, 101])
        self.assertEqual(check.call_count, 2)
        self.assertEqual({row.pid for row in self.session.members()}, {100, 101})
        snapshot, complete = self.session.process_snapshot(3)
        self.assertTrue(complete)
        self.assertEqual({row.pid for row in snapshot}, {100, 101, 102})
        for cap in (0, -1, 4097, 1.5, True):
            with self.subTest(cap=cap), self.assertRaises(ValueError):
                self.session.process_snapshot(cap)

    def test_diagnostic_snapshot_rejects_lost_leader_after_enumeration(self):
        row = scope.read_process(100, self.root)

        def inventory(_root):
            yield row
            self.write(100, start=8)

        with mock.patch.object(scope, 'process_rows', side_effect=inventory):
            with self.assertRaisesRegex(RuntimeError, 'identity'):
                self.session.process_snapshot(2)
        with self.assertRaisesRegex(RuntimeError, 'identity'):
            self.session.process_snapshot(2)

    def test_anchor_reuse_missing_session_or_parent_changes_reject(self):
        for values in (dict(start=8), dict(sid=101), dict(ppid=os.getpid() + 1)):
            with self.subTest(values=values):
                self.write(100, **values)
                with self.assertRaisesRegex(RuntimeError, 'identity'):
                    self.session.usage()
        (self.root / '100' / 'stat').unlink()
        with self.assertRaisesRegex(RuntimeError, 'identity'):
            self.session.members()

    def test_launch_requires_actual_direct_child_and_new_session(self):
        for values in (dict(sid=99), dict(pgid=99), dict(ppid=os.getpid() + 1)):
            self.write(100, **values)
            with self.assertRaisesRegex(RuntimeError, 'authenticate'):
                scope.OwnedSession(self.process, self.root)

    def test_exited_uses_nonreaping_waitid_and_preserves_child_status(self):
        with mock.patch.object(scope.os, 'waitid', return_value=None) as wait:
            self.assertFalse(self.session.exited())
            wait.assert_called_once_with(os.P_PID, 100,
                                         os.WEXITED | os.WNOHANG | os.WNOWAIT)
        with mock.patch.object(scope.os, 'waitid', return_value=object()):
            self.assertTrue(self.session.exited())
        self.process.poll.assert_not_called()
        self.process.wait.assert_not_called()

    def test_pidfd_checks_fresh_identity_session_and_zombie_before_signal(self):
        self.write(101)
        row = scope.read_process(101, self.root)
        for values in (dict(start=8), dict(sid=101), dict(state='Z'), dict(state='X')):
            with self.subTest(values=values):
                self.write(101, **values)
                with mock.patch.object(scope.os, 'pidfd_open', return_value=42), \
                        mock.patch.object(scope.signal, 'pidfd_send_signal') as send, \
                        mock.patch.object(scope.os, 'close') as close:
                    self.assertFalse(self.session._signal(row, signal.SIGTERM))
                    send.assert_not_called()
                    close.assert_called_once_with(42)

    def test_pidfd_exact_process_signal_and_exit_race_close_handles(self):
        self.write(101)
        row = scope.read_process(101, self.root)
        for vanished in (False, True):
            with self.subTest(vanished=vanished):
                with mock.patch.object(scope.os, 'pidfd_open', return_value=43) as opened, \
                        mock.patch.object(scope.signal, 'pidfd_send_signal',
                                          side_effect=ProcessLookupError if vanished else None) as send, \
                        mock.patch.object(scope.os, 'close') as close:
                    self.assertEqual(self.session._signal(row, signal.SIGKILL), not vanished)
                    opened.assert_called_once_with(101)
                    send.assert_called_once_with(43, signal.SIGKILL)
                    close.assert_called_once_with(43)
        with mock.patch.object(scope.os, 'pidfd_open', side_effect=ProcessLookupError):
            self.assertFalse(self.session._signal(row, signal.SIGTERM))

    def test_term_rescan_discovers_late_group_and_zombies_do_not_block_reap(self):
        self.write(101)
        delivered = []

        def send(row, signum):
            delivered.append((row.pid, signum))
            self.write(row.pid, state='Z')
            if row.pid == 101:
                self.write(102, ppid=1, pgid=102)
            return True

        with mock.patch.object(self.session, '_signal', side_effect=send), \
                mock.patch.object(scope.time, 'sleep'):
            self.session.stop()
        self.assertCountEqual(delivered[:2], [(100, signal.SIGTERM), (101, signal.SIGTERM)])
        self.assertEqual(delivered[2:], [(102, signal.SIGTERM)])
        self.process.wait.assert_called_once()
        self.assertEqual(self.session.record['cleanup'], 'complete')
        self.session.stop()  # Idempotent; does not inspect reaped numeric PID.
        self.process.wait.assert_called_once()

    def test_kill_rescans_and_cleanup_failure_never_reaps_identity_anchor(self):
        with mock.patch.object(self.session, '_drain', side_effect=[False, True]) as drain:
            self.session.stop()
        self.assertEqual(drain.call_args_list,
                         [mock.call(signal.SIGTERM, 2), mock.call(signal.SIGKILL, 2)])
        self.process.wait.assert_called_once()
        second = scope.OwnedSession(mock.Mock(pid=100), self.root)
        with mock.patch.object(second, '_drain', return_value=False) as drain:
            with self.assertRaisesRegex(RuntimeError, 'timed out'):
                second.stop()
            with self.assertRaisesRegex(RuntimeError, 'timed out'):
                second.stop()
            self.assertEqual(drain.call_count, 2)
        second.process.wait.assert_not_called()
        self.assertEqual(second.record['cleanup'], 'failed')

    def test_cleanup_term_is_once_per_observed_identity_and_wait_is_bounded(self):
        with mock.patch.object(self.session, '_signal', return_value=True) as send, \
                mock.patch.object(scope.time, 'monotonic', side_effect=[0, .1, 2]), \
                mock.patch.object(scope.time, 'sleep'):
            self.assertFalse(self.session._drain(signal.SIGTERM, 2))
        self.assertEqual(send.call_count, 1)

    def test_preflight_missing_api_rejects_and_supported_route_closes_handle(self):
        with mock.patch.object(scope.os, 'pidfd_open', None):
            with self.assertRaisesRegex(RuntimeError, 'requires Linux'):
                scope.preflight()
        with mock.patch.object(scope.os, 'pidfd_open', return_value=45), \
                mock.patch.object(scope.signal, 'pidfd_send_signal') as send, \
                mock.patch.object(scope.os, 'close') as close:
            scope.preflight()
        send.assert_called_once_with(45, 0)
        close.assert_called_once_with(45)


if __name__ == '__main__':
    unittest.main()
