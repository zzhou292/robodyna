"""Authenticate immutable fixture bytes independently of the current writer."""

import hashlib
import json
from pathlib import Path
import sys
import unittest


class FixtureIntegrity(unittest.TestCase):
    def test_frozen_bytes_and_pre_change_provenance(self):
        manifest = json.loads(MANIFEST.read_text())
        self.assertEqual(manifest["schema"], "robodyna.serialization-compatibility-baseline.v1")
        self.assertEqual(len(manifest["baseline_head"]), 40)
        self.assertEqual(set(manifest["files"]), {"baseline.json", "baseline.xml", "baseline.bin", "producer.json"})
        for name, expected in manifest["files"].items():
            self.assertEqual(Path(name).name, name)
            data = (MANIFEST.parent / name).read_bytes()
            self.assertGreater(len(data), 0)
            self.assertEqual(len(data), expected["bytes"], name)
            self.assertEqual(hashlib.sha256(data).hexdigest(), expected["sha256"], name)
        self.assertEqual(json.loads((MANIFEST.parent / "producer.json").read_text()), manifest["observed_producer"])
        self.assertGreaterEqual(len(manifest["production_inputs"]), 20)
        for record in manifest["production_inputs"].values():
            self.assertTrue(record["matches_recorded_head"])
            self.assertEqual(len(record["sha256"]), 64)
        # These are historical producer/input hashes, intentionally not rewritten
        # when the implementation changes after this baseline has been qualified.
        self.assertEqual(len(manifest["producer_binary"]["sha256"]), 64)


if __name__ == "__main__":
    MANIFEST = Path(sys.argv.pop(1))
    unittest.main()
