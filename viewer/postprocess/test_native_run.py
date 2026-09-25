"""Native schema dispatch uses real on-disk hash chains before mock rendering."""
import copy
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from .lifecycle import closed_run, read_json, sha256
from .job import run_job
from .native_run import SCHEMA, FORECAST_SCHEMA
from .test_replay_evidence import VALID_REPORT


class NativeCompletionTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        (self.root / 'archive').mkdir()
        self.launch = dict(guard_pid=42, guard_sid=42, guard_start_ticks=123,
                           guard_report=str(self.root / 'guard.json'), output=str(self.root))
        self.save('viewer-input.json', dict(schema='test receipt; C++ reader validates contents'))
        self.index = dict(accepted_intervals=1000, planned_intervals=1000, horizon_complete=True,
                          stop_reason='', final=dict(epoch=1000, time=0.0003),
                          frames=[dict(stamp=dict(epoch=0, time=0)),
                                  dict(stamp=dict(epoch=1000, time=0.0003))])
        self.configuration = dict(schema='robo_dyna.physical_run_configuration.v3', identity=dict(run=5),
            profile=dict(schema='robo_dyna.physical_observation_profile.v3',
                participants='qeph,t3,native_type25',
                native_contact='fixed_main_type25_accepted_base_force_history_v1'),
            intervals=1000, samples=31, fixed_dt_s=3e-7, requested_duration_s=.0003)
        self.summary = dict(schema=SCHEMA, caption='GPU shell-impact coupon',
            scope='accepted_visualization_not_vehicle_delivery_or_restart',
            forecast=dict(schema=FORECAST_SCHEMA, case='declared_gpu_shell_impact_coupon',
                scope='capacity_bounds_not_stability_or_completed_physics', run_id=5,
                planned_intervals=1000, samples=31, fixed_dt_s=3e-7, descriptive_duration_s=.0003),
            accepted_intervals=1000, actual_time_s=.0003, requested_steps_reached=True,
            stop_kind='completed', reason='', valid_closed_archive=True, output_error='')
        self.publish()

    def save(self, name, value):
        (self.root / name).write_text(json.dumps(value))

    def record(self, name):
        path = self.root / name
        return dict(file=path.name, bytes=path.stat().st_size, sha256=sha256(path))

    def publish(self, exit_code=0):
        self.save('guard.json', dict(exit_code=exit_code, process_scope=dict(cleanup='complete')))
        self.save('archive/frame-index.json', self.index)
        self.save('archive/configuration.json', self.configuration)
        self.save('archive/manifest.json', dict(index=self.record('archive/frame-index.json'),
            configuration=self.record('archive/configuration.json')))
        self.summary['viewer_input'] = self.record('viewer-input.json')
        self.summary['archive_manifest'] = self.record('archive/manifest.json')
        self.save('summary.json', self.summary)

    def prefix(self):
        self.summary.update(accepted_intervals=473, actual_time_s=.0001419,
            requested_steps_reached=False, stop_kind='requested_stop', reason='Operator stop')
        self.index.update(accepted_intervals=473, horizon_complete=False, stop_reason='Operator stop',
                          final=dict(epoch=473, time=.0001419))
        self.index['frames'][-1]['stamp'] = dict(epoch=473, time=.0001419)
        self.publish(2)

    def test_complete_and_honest_prefix_normalize_without_mutating_summary(self):
        for prefix in (False, True):
            if prefix:
                self.prefix()
            before = (self.root / 'summary.json').read_bytes()
            summary, index, digest = closed_run(self.launch)
            self.assertEqual(summary['schema'], SCHEMA)
            self.assertEqual(summary['_summary_file'], 'summary.json')
            self.assertEqual(summary['actual_completed_time_s'], self.summary['actual_time_s'])
            self.assertEqual(index, self.index)
            self.assertEqual(digest, self.record('viewer-input.json')['sha256'])
            self.assertEqual((self.root / 'summary.json').read_bytes(), before)
            self.assertFalse((self.root / 'run-summary.json').exists())

    def test_native_schema_and_typed_claims_cannot_authorize_another_profile(self):
        original = copy.deepcopy(self.summary)
        changes = [('schema', 'unknown'), ('valid_closed_archive', 1), ('output_error', 'failed'),
                   ('accepted_intervals', True), ('accepted_intervals', 1001),
                   ('actual_time_s', float('nan')), ('requested_steps_reached', 1),
                   ('requested_steps_reached', False), ('reason', 'unexpected'),
                   ('stop_kind', 'archive_failure'), ('stop_kind', 'invented')]
        for key, value in changes:
            with self.subTest(key=key, value=value):
                self.summary = {**copy.deepcopy(original), key: value}
                self.publish()
                with self.assertRaises(ValueError):
                    closed_run(self.launch)
        self.summary = copy.deepcopy(original)
        for key, value in [('schema', 'foreign'), ('case', 'vehicle'), ('scope', 'physical_pass'),
                           ('run_id', False), ('planned_intervals', True), ('samples', 0), ('samples', 1),
                           ('samples', 1001),
                           ('fixed_dt_s', float('inf')), ('descriptive_duration_s', 0)]:
            with self.subTest(forecast=key):
                self.summary = copy.deepcopy(original)
                self.summary['forecast'][key] = value
                self.publish()
                with self.assertRaises(ValueError):
                    closed_run(self.launch)

    def test_guard_exit_and_index_endpoint_must_match_complete_or_prefix(self):
        for prefix in (False, True):
            if prefix:
                self.prefix()
            saved = copy.deepcopy(self.index)
            for field, value in [('accepted_intervals', True), ('planned_intervals', 999),
                                 ('horizon_complete', 1), ('stop_reason', 'wrong'),
                                 ('final', dict(epoch=17, time=0))]:
                with self.subTest(prefix=prefix, field=field):
                    self.index = {**copy.deepcopy(saved), field: value}
                    self.publish(2 if prefix else 0)
                    with self.assertRaises(ValueError):
                        closed_run(self.launch)
            self.index = copy.deepcopy(saved)
            self.index['frames'][-1]['stamp']['time'] += 1e-9
            self.publish(2 if prefix else 0)
            with self.assertRaises(ValueError):
                closed_run(self.launch)
            self.index = saved
            self.publish(0 if prefix else 2)
            with self.assertRaises(ValueError):
                closed_run(self.launch)

    def test_false_exit_and_unknown_prefix_reason_kind_do_not_count_as_success(self):
        self.publish(False)
        with self.assertRaises(ValueError):
            closed_run(self.launch)
        self.prefix()
        self.summary['stop_kind'] = 'invented'
        self.publish(2)
        with self.assertRaises(ValueError):
            closed_run(self.launch)

    def test_forecast_cannot_relabel_a_different_authenticated_archive(self):
        original = copy.deepcopy(self.configuration)
        changes = [('schema', 'robo_dyna.physical_run_configuration.v2'),
                   ('intervals', 900), ('samples', 30), ('fixed_dt_s', 2e-7),
                   ('requested_duration_s', .0002), ('identity', dict(run=6))]
        for key, value in changes:
            with self.subTest(key=key):
                self.configuration = {**copy.deepcopy(original), key: value}
                self.publish()
                with self.assertRaises(ValueError):
                    closed_run(self.launch)
        for key, value in [('schema', 'robo_dyna.physical_observation_profile.v2'),
                           ('participants', 'qeph,t3,qbat,type25,type13,solids'),
                           ('native_contact', 'other')]:
            with self.subTest(profile=key):
                self.configuration = copy.deepcopy(original)
                self.configuration['profile'][key] = value
                self.publish()
                with self.assertRaises(ValueError):
                    closed_run(self.launch)

    def test_record_name_length_hash_and_bytes_are_authenticated(self):
        for name in ('viewer-input.json', 'archive/manifest.json', 'archive/frame-index.json',
                     'archive/configuration.json'):
            with self.subTest(changed=name):
                path = self.root / name
                old = path.read_bytes()
                path.write_bytes(old + b' ')
                with self.assertRaises(ValueError):
                    closed_run(self.launch)
                path.write_bytes(old)
        original = copy.deepcopy(self.summary)
        for field, value in [('file', '../viewer-input.json'), ('bytes', True), ('bytes', 999),
                             ('sha256', '0' * 64), ('sha256', 'NOT A HASH')]:
            with self.subTest(record=field):
                self.summary = copy.deepcopy(original)
                self.summary['viewer_input'][field] = value
                self.save('summary.json', self.summary)
                with self.assertRaises(ValueError):
                    closed_run(self.launch)

    def test_native_job_hashes_actual_summary_and_keeps_generic_notifications(self):
        self.save('launch.json', self.launch)
        directory = self.root / 'job'
        config = dict(job_directory=str(directory), launch_receipt=str(self.root / 'launch.json'),
            run=str(self.root), wait_timeout_s=1, requested_steps=1000, pinned_files={},
            environment={}, scene_checker='checker', viewer='viewer', ffmpeg='ffmpeg',
            views=[dict(name='overview', camera_arguments=[])])

        def step(config, directory, label, command, gpu=False):
            if label == 'archive-replay':
                (directory / 'archive-replay.xml').write_text(VALID_REPORT)
            elif label == 'overview-encode':
                output = directory / 'overview-video'
                output.mkdir()
                (output / 'review.mp4').write_bytes(b'mock')

        with patch('viewer.postprocess.job.wait_for_run', return_value=closed_run(self.launch)), \
             patch('viewer.postprocess.job.bounded_command', side_effect=step), \
             patch('viewer.postprocess.job.notify', return_value=dict(sent=True)) as notify:
            run_job(config)
        status = read_json(directory / 'status.json')
        self.assertEqual(status['stage'], 'complete')
        self.assertEqual(status['summary_file'], 'summary.json')
        self.assertEqual(status['run_summary_sha256'], sha256(self.root / 'summary.json'))
        self.assertEqual(status['simulated_time_s'], .0003)
        self.assertEqual(notify.call_count, 2)
        for call in notify.call_args_list:
            self.assertNotIn('Yaris', ' '.join(call.args[:2]))
            self.assertIn(self.root.name, call.args[1])
            self.assertIn('GPU shell-impact coupon', call.args[1])


if __name__ == '__main__':
    unittest.main()
