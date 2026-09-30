import json
from pathlib import Path
import tempfile
import unittest

from compare import compare, sha256
from raw_json import read_object
from main import check_report_location


class PayloadComparison(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.old, self.new = self.root / "old", self.root / "new"
        for run in (self.old, self.new):
            (run / "archive").mkdir(parents=True)
            (run / "archive/manifest.json").write_text('{}')
            (run / "archive/frame.bin").write_bytes(b'\x00\x80')
            (run / "viewer-input.json").write_text('{}')
            data = dict(schema="robo_dyna.vehicle_run_summary.v1", session_initialized=True,
                        valid_archive_manifest=True, accepted_intervals=2,
                        complete_host_upper_bound=100, complete_archive_upper_bound=20,
                        successful_prepare_wall_s=1, elapsed_after_startup_s=2,
                        accepted_intervals_per_second=1, accepted_mechanics={"force": -0.0},
                        sampled_shell_plasticity={}, archive_manifest_sha256=sha256(run / "archive/manifest.json"),
                        viewer_input_sha256=sha256(run / "viewer-input.json"))
            (run / "run-summary.json").write_text(json.dumps(data))

    def mutate(self, key, value):
        path = self.new / "run-summary.json"
        data = json.loads(path.read_text()); data[key] = value
        path.write_text(json.dumps(data))

    def test_timing_and_declared_forecast_change_only(self):
        self.mutate("elapsed_after_startup_s", 1.1)
        self.mutate("complete_host_upper_bound", 109)
        with self.assertRaises(ValueError):
            compare(self.old, self.new)
        result = compare(self.old, self.new, 9)
        self.assertEqual(result["archive_files"], 2)

    def test_signed_zero_and_numeric_spelling_are_preserved(self):
        for replacement in ('0.0', '-0e0'):
            p = self.new / "run-summary.json"
            p.write_text((self.old / "run-summary.json").read_text().replace('-0.0', replacement))
            with self.assertRaisesRegex(ValueError, 'accepted_mechanics'):
                compare(self.old, self.new)

    def test_payload_bit_flip_and_additional_file_rejected(self):
        p = self.new / "archive/frame.bin"; p.write_bytes(b'\x00\x00')
        with self.assertRaisesRegex(ValueError, 'payload differs'):
            compare(self.old, self.new)
        p.write_bytes(b'\x00\x80'); (self.new / "archive/extra").write_bytes(b'')
        with self.assertRaises(ValueError):
            compare(self.old, self.new)

    def test_unknown_summary_field_is_not_ignored(self):
        self.mutate("new_diagnostic", 1)
        with self.assertRaisesRegex(ValueError, 'field inventory'):
            compare(self.old, self.new)

    def test_both_missing_required_diagnostic_rejected(self):
        for run in (self.old, self.new):
            path = run / "run-summary.json"
            data = json.loads(path.read_text())
            del data['accepted_mechanics']
            path.write_text(json.dumps(data))
        with self.assertRaises(ValueError):
            compare(self.old, self.new)

    def test_digest_and_failed_or_empty_run_rejected(self):
        for key, value in [('archive_manifest_sha256', 'bad'), ('valid_archive_manifest', False),
                           ('accepted_intervals', 0)]:
            (self.new / "run-summary.json").write_bytes((self.old / "run-summary.json").read_bytes())
            self.mutate(key, value)
            with self.assertRaises(ValueError):
                compare(self.old, self.new)

    def test_json_ambiguity_rejected_and_nested_key_not_confused(self):
        p = self.root / "sample.json"
        for content in ['{"a":1,"a":2}', '{"a":{"b":1,"b":2}}', '{"a":NaN}']:
            p.write_text(content)
            with self.assertRaises(ValueError):
                read_object(p)
        p.write_text('{"nested":{"a":-0.0}, "a":1e-3}')
        self.assertEqual(read_object(p)[1]['a'], '1e-3')

    def test_report_cannot_modify_either_run(self):
        check_report_location(self.root / 'report.json', self.old, self.new)
        for run in (self.old, self.new):
            with self.assertRaises(ValueError):
                check_report_location(run / 'archive/report.json', self.old, self.new)
        (self.root / 'alias').symlink_to(self.old, target_is_directory=True)
        with self.assertRaises(ValueError):
            check_report_location(self.root / 'alias/new.json', self.old, self.new)


if __name__ == "__main__":
    unittest.main()
