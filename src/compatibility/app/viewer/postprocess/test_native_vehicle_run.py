"""Closure-only hash-chain fixtures; C++ still validates every real frame."""
import copy
import json
from pathlib import Path
import tempfile
import unittest

from .lifecycle import closed_run, sha256
from .native_vehicle_run import SCHEMA, PARTICIPANTS


class NativeVehicleCompletionTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        (self.root / 'archive').mkdir()
        self.launch = dict(output=str(self.root), guard_report=str(self.root / 'guard.json'))
        self.summary = dict(schema=SCHEMA,
            physical_profile='selected_vehicle_supports_v5_with_declared_finite_mesh_wall',
            contact_profile='source_type25_self_and_all_retained_nodes_to_fixed_mesh',
            initial_state='source_produced_starter_history_and_final_type2_removals',
            visualization_only_not_restart=True, session_initialized=True, valid_closed_archive=True,
            horizon_complete=True, planned_intervals=25000, accepted_intervals=25000,
            actual_time_s=.005, fixed_dt_s=2e-7, requested_duration_s=.005,
            stop_kind=0, stop_reason='', interfaces=[
                dict(role='self', native_id=1, native_storage_ordinal=1, accepted_intervals=25000,
                     active_force_intervals=25000, initialization_available=True),
                dict(role='mesh_wall', native_id=10001073, native_storage_ordinal=3,
                     accepted_intervals=25000, active_force_intervals=100, initialization_available=True)])
        self.configuration = dict(schema='robo_dyna.physical_run_configuration.v5',
            environment_wall='native_declared_fixed_elastic_wall_v1',
            profile=dict(schema='robo_dyna.physical_observation_profile.v4',
                purpose='selected_physical_model_accepted_visualization_not_restart',
                participants=PARTICIPANTS, native_group='declared_order_common_owner_accepted_publications_v1'),
            intervals=25000, samples=61, fixed_dt_s=2e-7, requested_duration_s=.005)
        self.index = dict(schema='robo_dyna.physical_accepted_index.v1',
            planned_intervals=25000, accepted_intervals=25000, horizon_complete=True, stop_reason='',
            final=dict(epoch=25000, time=.005), frames=[dict(stamp=dict(epoch=0, time=0)),
                dict(stamp=dict(epoch=25000, time=.005))])
        self.publish()

    def save(self, name, values):
        (self.root / name).write_text(json.dumps(values))

    def record(self, name):
        path = self.root / name
        return dict(file=path.name, bytes=path.stat().st_size, sha256=sha256(path))

    def publish(self, exit_code=0):
        self.save('guard.json', dict(exit_code=exit_code, process_scope=dict(cleanup='complete')))
        self.save('archive/configuration.json', self.configuration)
        self.save('archive/frame-index.json', self.index)
        self.save('archive/manifest.json', dict(schema='robo_dyna.physical_accepted_run.v2',
            configuration=self.record('archive/configuration.json'), index=self.record('archive/frame-index.json')))
        self.summary['archive_manifest'] = self.record('archive/manifest.json')
        self.save('viewer-input.json', dict(schema='robo_dyna.physical_viewer_input.v1',
            archive_directory='archive', manifest=self.summary['archive_manifest']))
        self.summary['viewer_input'] = self.record('viewer-input.json')
        self.save('summary.json', self.summary)

    def prefix(self, exit_code=2):
        self.summary.update(horizon_complete=False, accepted_intervals=123, actual_time_s=.0000246,
                            stop_kind=5, stop_reason='Authentic candidate rejected')
        for row in self.summary['interfaces']:
            row.update(accepted_intervals=123, active_force_intervals=12)
        self.index.update(horizon_complete=False, accepted_intervals=123,
                          stop_reason=self.summary['stop_reason'], final=dict(epoch=123, time=.0000246))
        self.index['frames'][-1]['stamp'] = dict(epoch=123, time=.0000246)
        self.publish(exit_code)

    def test_complete_and_typed_prefix_reuse_operator_view_without_file_mutation(self):
        for prefix in (False, True):
            if prefix:
                self.prefix()
            before = {p: p.read_bytes() for p in self.root.rglob('*.json')}
            summary, index, digest = closed_run(self.launch)
            self.assertEqual(summary['schema'], SCHEMA)
            self.assertEqual(summary['_summary_file'], 'summary.json')
            self.assertEqual(summary['_display_profile'], 'V5 vehicle + finite mesh wall')
            self.assertEqual(summary['reason'], self.summary['stop_reason'])
            self.assertEqual(summary['actual_completed_time_s'], self.summary['actual_time_s'])
            self.assertEqual(index, self.index)
            self.assertEqual(digest, self.record('viewer-input.json')['sha256'])
            self.assertEqual(before, {p: p.read_bytes() for p in self.root.rglob('*.json')})
            self.assertFalse((self.root / 'run-summary.json').exists())

    def test_failed_owning_qualification_is_not_promoted_by_a_closed_prefix(self):
        self.prefix(1)
        with self.assertRaises(ValueError):
            closed_run(self.launch)
        self.publish(False)
        with self.assertRaises(ValueError):
            closed_run(self.launch)
        self.publish(2)
        self.save('guard.json', dict(exit_code=2, process_scope=dict(cleanup='incomplete')))
        with self.assertRaises(ValueError):
            closed_run(self.launch)

    def test_native_v6_profile_keeps_source_label_and_all_existing_hash_checks(self):
        self.summary['physical_profile'] = 'native_v6_raw8_heph_explicit_cin28_with_declared_finite_mesh_wall'
        self.publish()
        before = {p: p.read_bytes() for p in self.root.rglob('*.json')}
        summary, _, _ = closed_run(self.launch)
        self.assertEqual(summary['physical_profile'], self.summary['physical_profile'])
        self.assertEqual(summary['_display_profile'], 'Native V6 vehicle + finite mesh wall')
        self.assertEqual(before, {p: p.read_bytes() for p in self.root.rglob('*.json')})
        with (self.root / 'archive/configuration.json').open('a') as stream:
            stream.write(' ')
        with self.assertRaises(ValueError):
            closed_run(self.launch)

    def test_typed_summary_and_completion_reject_inconsistent_claims(self):
        before = copy.deepcopy(self.summary)
        for key, value in [('schema', 'robo_dyna.native_shell_impact_run.v1'),
                           ('physical_profile', 'unqualified_vehicle_profile'),
                           ('valid_closed_archive', 1), ('session_initialized', False),
                           ('visualization_only_not_restart', False), ('accepted_intervals', True),
                           ('accepted_intervals', 25001), ('actual_time_s', float('nan')),
                           ('horizon_complete', 1), ('horizon_complete', False),
                           ('stop_kind', True), ('stop_kind', 4), ('stop_kind', 6),
                           ('stop_reason', 'false completion'), ('viewer_error', 'failed')]:
            with self.subTest(key=key, value=value):
                self.summary = {**copy.deepcopy(before), key: value}
                self.publish()
                with self.assertRaises((ValueError, KeyError)):
                    closed_run(self.launch)

    def test_rehashed_profile_and_index_changes_still_reject(self):
        config, index = copy.deepcopy(self.configuration), copy.deepcopy(self.index)
        changes = [(('schema',), 'robo_dyna.physical_run_configuration.v3'),
                   (('environment_wall',), 'legacy_wall'), (('intervals',), 24999),
                   (('samples',), True), (('samples',), 1001), (('fixed_dt_s',), 3e-7),
                   (('requested_duration_s',), .006), (('profile', 'participants'), 'qeph,t3,native_type25'),
                   (('profile', 'native_group'), 'invented'), (('profile', 'native_contact'), 'legacy')]
        for keys, value in changes:
            with self.subTest(keys=keys):
                self.configuration = copy.deepcopy(config)
                target = self.configuration
                for key in keys[:-1]:
                    target = target[key]
                target[keys[-1]] = value
                self.publish()
                with self.assertRaises(ValueError):
                    closed_run(self.launch)
        self.configuration = config
        for key, value in [('planned_intervals', 25001), ('accepted_intervals', 24999),
                           ('horizon_complete', False), ('stop_reason', 'unexpected'), ('frames', [])]:
            with self.subTest(index=key):
                self.index = {**copy.deepcopy(index), key: value}
                self.publish()
                with self.assertRaises(ValueError):
                    closed_run(self.launch)
        self.index = copy.deepcopy(index)
        self.index['frames'][-1]['stamp']['time'] = .0049
        self.publish()
        with self.assertRaises(ValueError):
            closed_run(self.launch)

    def test_interface_identity_order_and_accepted_counters_are_not_inferred(self):
        baseline = copy.deepcopy(self.summary)
        for key, value in [('role', 'self'), ('native_id', 1), ('native_storage_ordinal', 1),
                           ('accepted_intervals', 24999), ('active_force_intervals', 25001),
                           ('initialization_available', False)]:
            with self.subTest(key=key):
                self.summary = copy.deepcopy(baseline)
                self.summary['interfaces'][1][key] = value
                self.publish()
                with self.assertRaises(ValueError):
                    closed_run(self.launch)
        self.summary = copy.deepcopy(baseline)
        self.summary['interfaces'].reverse()
        self.publish()
        with self.assertRaises(ValueError):
            closed_run(self.launch)

    def test_record_hashes_paths_and_viewer_manifest_link_remain_authoritative(self):
        for name in ('viewer-input.json', 'archive/manifest.json', 'archive/configuration.json', 'archive/frame-index.json'):
            with self.subTest(file=name):
                self.publish()
                with (self.root / name).open('a') as stream:
                    stream.write(' ')
                with self.assertRaises(ValueError):
                    closed_run(self.launch)
        self.publish()
        self.summary['viewer_input']['file'] = '../viewer-input.json'
        self.save('summary.json', self.summary)
        with self.assertRaises(ValueError):
            closed_run(self.launch)
        self.publish()
        viewer = json.loads((self.root / 'viewer-input.json').read_text())
        viewer['manifest']['sha256'] = '0' * 64
        self.save('viewer-input.json', viewer)
        self.summary['viewer_input'] = self.record('viewer-input.json')
        self.save('summary.json', self.summary)
        with self.assertRaises(ValueError):
            closed_run(self.launch)


if __name__ == '__main__':
    unittest.main()
