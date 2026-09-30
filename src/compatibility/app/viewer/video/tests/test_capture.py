import copy
import json
from pathlib import Path
import tempfile
import unittest

from viewer.video.capture import metadata_dict, validate_capture
from viewer.video.tests.fixtures import capture, write_index


class CaptureTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.directory = Path(self.temp.name) / "capture"
        self.rows, self.metadata = capture(self.directory)

    def test_normal_and_recovered_preserve_time_and_authority(self):
        normal = validate_capture(self.directory)
        self.assertEqual([frame.epoch for frame in normal.frames], [0, 25, 27])
        self.assertFalse(normal.metadata["input_horizon_complete"])
        with self.assertRaises(TypeError):
            normal.metadata["input_receipt"]["file"] = "changed"
        recovered_dir = self.directory.parent / "recovered"
        _, metadata = capture(recovered_dir, recovered=True)
        recovered = validate_capture(recovered_dir)
        self.assertEqual(metadata_dict(recovered), metadata)
        self.assertNotIn("input_horizon_complete", recovered.metadata)
        self.assertFalse(recovered.metadata["interval_ledger_available"])

    def test_reject_capture_contract_and_final_stamp_tampering(self):
        mutations = {"complete_capture": False, "interpolated_frames": True,
                     "deformation_scale": 10, "simulation_executed_by_viewer": True,
                     "all_png_decoded": False, "frames": 4, "png_bytes": 1,
                     "width": 4, "height": 3, "final_epoch": 26,
                     "final_time_s": 0.0000055, "schema": "unknown"}
        for key, value in mutations.items():
            with self.subTest(key=key):
                changed = {**self.metadata, key: value}
                write_index(self.directory, self.rows, changed)
                with self.assertRaises(ValueError):
                    validate_capture(self.directory)

    def test_reject_order_path_digest_and_nonmonotonic_rows(self):
        mutations = [(1, "sample", 0), (1, "file", "../frame-000001.png"),
                     (1, "epoch", 0), (1, "accepted_time_s", 0),
                     (1, "accepted_time_s", "NaN"), (1, "accepted_time_s", "inf"),
                     (1, "sha256", "0" * 64), (1, "bytes", 999),
                     (2, "file", "frame-000001.png")]
        for row, key, value in mutations:
            with self.subTest(key=key, value=value):
                changed = copy.deepcopy(self.rows)
                changed[row][key] = value
                write_index(self.directory, changed, dict(self.metadata))
                with self.assertRaises(ValueError):
                    validate_capture(self.directory)

    def test_reject_index_and_same_size_png_mutations(self):
        with (self.directory / "frames.csv").open("a") as stream:
            stream.write("\n")
        with self.assertRaisesRegex(ValueError, "digest"):
            validate_capture(self.directory)
        write_index(self.directory, self.rows, self.metadata)
        image = self.directory / self.rows[1]["file"]
        data = bytearray(image.read_bytes())
        data[-1] ^= 1
        image.write_bytes(data)
        with self.assertRaisesRegex(ValueError, "digest"):
            validate_capture(self.directory)

    def test_reject_duplicate_manifest_keys_and_unavailable_recovery_claims(self):
        path = self.directory / "manifest.json"
        path.write_text(path.read_text()[:-1] + ', "complete_capture": true}')
        with self.assertRaisesRegex(ValueError, "Duplicate"):
            validate_capture(self.directory)
        recovered_dir = self.directory.parent / "recovered"
        rows, metadata = capture(recovered_dir, recovered=True)
        metadata["interval_ledger_available"] = True
        write_index(recovered_dir, rows, metadata)
        with self.assertRaisesRegex(ValueError, "Recovered"):
            validate_capture(recovered_dir)


if __name__ == "__main__":
    unittest.main()
