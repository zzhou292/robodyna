"""Small fixtures only; never read or hash a real simulation archive."""

import hashlib
import json
from pathlib import Path
import tempfile
import unittest

from tools.migration.verify_run import authenticate_inventory, record_path, require_closed_guard, summary_differences


class ArchiveComparisonTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.archive = self.root / "archive"
        self.archive.mkdir()
        data = b"accepted physics bytes"
        (self.archive / "frame.bin").write_bytes(data)
        self.manifest = {"files": [{"file": "frame.bin", "bytes": len(data), "sha256": hashlib.sha256(data).hexdigest()}]}
        (self.archive / "manifest.json").write_text(json.dumps(self.manifest))

    def test_authenticates_complete_inventory(self):
        result = authenticate_inventory(self.root)
        self.assertEqual(set(result), {"frame.bin", "manifest.json"})
        self.assertEqual(result["frame.bin"]["sha256"], self.manifest["files"][0]["sha256"])

    def test_rejects_same_size_changed_physics(self):
        path = self.archive / "frame.bin"
        path.write_bytes(b"X" * path.stat().st_size)
        with self.assertRaisesRegex(ValueError, "hash differs"):
            authenticate_inventory(self.root)

    def test_rejects_extra_and_missing_records(self):
        (self.archive / "extra.bin").write_bytes(b"extra")
        with self.assertRaisesRegex(ValueError, "inventory differs"):
            authenticate_inventory(self.root)
        (self.archive / "extra.bin").unlink()
        (self.archive / "frame.bin").unlink()
        with self.assertRaisesRegex(ValueError, "inventory differs"):
            authenticate_inventory(self.root)

    def test_rejects_record_path_escape_and_symlink(self):
        with self.assertRaisesRegex(ValueError, "canonical and relative"):
            record_path(self.archive, "../outside.bin")
        (self.archive / "link.bin").symlink_to(self.archive / "frame.bin")
        with self.assertRaisesRegex(ValueError, "symlink"):
            authenticate_inventory(self.root)

    def test_rejects_duplicate_inventory_identity(self):
        self.manifest["files"].append(self.manifest["files"][0])
        (self.archive / "manifest.json").write_text(json.dumps(self.manifest))
        with self.assertRaisesRegex(ValueError, "duplicate"):
            authenticate_inventory(self.root)

    def test_requires_closed_guard_before_accepting_output(self):
        guard = self.root / "guard.json"
        guard.write_text(json.dumps({"exit_code": 0, "process_scope": {"cleanup": "pending"}}))
        with self.assertRaisesRegex(ValueError, "completed"):
            require_closed_guard(guard)
        guard.write_text(json.dumps({"exit_code": 125, "process_scope": {"cleanup": "complete"}}))
        with self.assertRaisesRegex(ValueError, "completed"):
            require_closed_guard(guard)

    def test_only_named_timings_are_excluded_from_exact_summary(self):
        other = self.root / "candidate"
        other.mkdir()
        (self.root / "summary.json").write_text('{"step_s":1.0,"mechanics":{"x":0.0}}')
        (other / "summary.json").write_text('{"step_s":2.0,"mechanics":{"x":0.0}}')
        self.assertEqual(summary_differences(self.root, other), ([], ["step_s"]))
        (other / "summary.json").write_text('{"step_s":2.0,"mechanics":{"x":-0.0}}')
        self.assertEqual(summary_differences(self.root, other), (["mechanics"], ["step_s"]))


if __name__ == "__main__":
    unittest.main()
