import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from .lifecycle import closed_run, guard_alive, read_json, sha256, wait_for_run
from .job import bounded_command, run_job


class CompletionTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        (self.root / 'archive').mkdir()
        self.launch = dict(guard_pid=42, guard_sid=42, guard_start_ticks=123,
                           guard_report=str(self.root / 'guard.json'), output=str(self.root))
        self.save('guard.json', dict(exit_code=2, process_scope=dict(cleanup='complete')))
        self.save('archive/frame-index.json', dict(accepted_intervals=25, frames=[{'stamp': 0}]))
        self.save('archive/manifest.json', dict(index=dict(sha256=sha256(self.root / 'archive/frame-index.json'))))
        self.save('viewer-input.json', dict(schema='receipt'))
        self.summary = dict(valid_archive_manifest=True, accepted_intervals=25,
                            actual_completed_time_s=5e-6, reason='Declared limit',
                            viewer_input_sha256=sha256(self.root / 'viewer-input.json'),
                            archive_manifest_sha256=sha256(self.root / 'archive/manifest.json'))
        self.save('run-summary.json', self.summary)

    def save(self, name, data):
        (self.root / name).write_text(json.dumps(data))

    def test_normal_diagnostic_exit_two_is_renderable(self):
        summary, index, receipt = closed_run(self.launch)
        self.assertEqual(summary['accepted_intervals'], 25)
        self.assertEqual(len(index['frames']), 1)
        self.assertEqual(receipt, self.summary['viewer_input_sha256'])

    def test_rejects_missing_cleanup_bad_exit_and_invalid_publication(self):
        for field in ('cleanup', 'exit', 'publication'):
            with self.subTest(field=field):
                self.save('guard.json', dict(exit_code=7 if field == 'exit' else 2,
                    process_scope=dict(cleanup='failed' if field == 'cleanup' else 'complete')))
                self.save('run-summary.json', {**self.summary, 'valid_archive_manifest': field != 'publication'})
                with self.assertRaises(ValueError):
                    closed_run(self.launch)

    def test_rejects_modified_index_receipt_or_manifest(self):
        for name in ('archive/frame-index.json', 'viewer-input.json', 'archive/manifest.json'):
            with self.subTest(name=name):
                path = self.root / name
                original = path.read_bytes()
                path.write_bytes(original + b' ')
                with self.assertRaises(ValueError):
                    closed_run(self.launch)
                path.write_bytes(original)

    def test_guard_identity_rejects_reused_pid_and_zombie(self):
        pid = self.root / '42'
        pid.mkdir()
        fields = ['0'] * 20
        fields[0], fields[3], fields[19] = 'S', '42', '123'
        stat = pid / 'stat'
        stat.write_text('42 (guard with spaces) ' + ' '.join(fields))
        self.assertTrue(guard_alive(self.launch, self.root))
        fields[19] = '124'
        stat.write_text('42 (guard) ' + ' '.join(fields))
        self.assertFalse(guard_alive(self.launch, self.root))
        fields[19], fields[0] = '123', 'Z'
        stat.write_text('42 (guard) ' + ' '.join(fields))
        self.assertFalse(guard_alive(self.launch, self.root))

    def test_wait_does_not_open_output_while_guard_runs(self):
        with patch('viewer.postprocess.lifecycle.guard_alive', side_effect=[True, False]), \
             patch('viewer.postprocess.lifecycle.time.sleep') as sleep, \
             patch('viewer.postprocess.lifecycle.closed_run', return_value='ready') as closed:
            self.assertEqual(wait_for_run(self.launch, 60), 'ready')
            sleep.assert_called_once()
            closed.assert_called_once_with(self.launch)

    def test_timeout_does_not_signal_or_read_running_simulation(self):
        with patch('viewer.postprocess.lifecycle.guard_alive', return_value=True), \
             patch('viewer.postprocess.lifecycle.closed_run') as closed:
            with self.assertRaises(TimeoutError):
                wait_for_run(self.launch, 0)
            closed.assert_not_called()

    def test_missing_final_report_does_not_infer_success(self):
        (self.root / 'guard.json').unlink()
        with self.assertRaises(FileNotFoundError):
            closed_run(self.launch)

    def test_job_preserves_first_video_when_later_capture_fails(self):
        self.save('launch.json', self.launch)
        directory = self.root / 'job'
        config = dict(job_directory=str(directory), launch_receipt=str(self.root / 'launch.json'),
            run=str(self.root), wait_timeout_s=1, requested_steps=25, pinned_files={},
            environment={}, scene_checker='checker', viewer='viewer', ffmpeg='ffmpeg',
            views=[dict(name='overview', camera_arguments=[]), dict(name='front', camera_arguments=[])])

        def step(config, directory, label, command, gpu=False):
            if label == 'archive-replay':
                (directory / 'archive-replay.xml').write_text(
                    '<testsuites tests="1" failures="0" errors="0" disabled="0"/>')
            elif label == 'overview-encode':
                dest = directory / 'overview-video'
                dest.mkdir()
                (dest / 'review.mp4').write_bytes(b'mock')
            elif label == 'front-capture':
                raise RuntimeError('capture failed')

        with patch('viewer.postprocess.job.wait_for_run', return_value=closed_run(self.launch)), \
             patch('viewer.postprocess.job.bounded_command', side_effect=step), \
             patch('viewer.postprocess.job.notify', return_value=dict(sent=True)):
            with self.assertRaisesRegex(RuntimeError, 'capture failed'):
                run_job(config)
        status = read_json(directory / 'status.json')
        self.assertEqual(status['stage'], 'failed')
        self.assertEqual(len(status['videos']), 1)
        self.assertTrue(Path(status['videos'][0]).is_file())

    def test_bounded_stage_uses_shared_lock_and_explicit_limits(self):
        config = dict(guard='guard', workstation_lock='shared.lock', environment={},
                      application='app', run='run', workspace='workspace')
        with patch('viewer.postprocess.job.subprocess.run') as execute, \
             patch('viewer.postprocess.job.read_json', return_value={
                 'status': 'passed', 'process_scope': {'cleanup': 'complete'}}):
            bounded_command(config, self.root, 'capture', ['viewer', 'input'], gpu=True)
        args = execute.call_args.args[0]
        for option, value in (('--lock', 'shared.lock'), ('--cpus', '2'),
                              ('--max-rss-gib', '10'), ('--min-available-gib', '32'),
                              ('--gpu', '0'), ('--max-gpu-growth-gib', '6')):
            self.assertEqual(args[args.index(option) + 1], value)
        self.assertEqual(args[-3:], ['--', 'viewer', 'input'])


if __name__ == '__main__':
    unittest.main()
