"""Verify that optional profiles add only their retained terrain implementations."""

import hashlib
import json
from pathlib import Path
import sys
import unittest

from src.vehicle.CMakeSources import PROFILE, library_groups, source_groups


def implementations(profile):
    groups = source_groups(CMAKE, "src/chrono_vehicle", "CV", profile)
    return {path for values in library_groups(CMAKE, "Chrono_vehicle", groups).values()
            for path in values if path.endswith(".cpp")}


class VehicleOptionalSources(unittest.TestCase):
    def test_optional_flags_add_only_the_two_actual_terrain_implementations(self):
        baseline = implementations(PROFILE)
        crm = implementations(dict(PROFILE, CH_ENABLE_MODULE_FSI_SPH=True))
        crg = implementations(dict(PROFILE, CH_USE_OPENCRG=True))
        combined = implementations(dict(PROFILE, CH_ENABLE_MODULE_FSI_SPH=True, CH_USE_OPENCRG=True))
        self.assertEqual(crm - baseline, {"src/chrono_vehicle/terrain/CRMTerrain.cpp"})
        self.assertEqual(crg - baseline, {"src/chrono_vehicle/terrain/CRGTerrain.cpp"})
        self.assertEqual(combined, crm | crg)
        self.assertFalse(baseline - crm)
        self.assertFalse(baseline - crg)
        self.assertIn("src/chrono_vehicle/wheeled_vehicle/test_rig/ChWheelTestRig.cpp", baseline & crm)

    def test_terrain_rig_and_road_input_bytes_are_unchanged(self):
        root = CMAKE.parents[2]
        document = json.loads(BASELINE.read_text())
        self.assertEqual(document["schema"], "robodyna.vehicle_optional_sources.v1")
        for entry in document["files"]:
            with self.subTest(path=entry["path"]):
                payload = (root / entry["path"]).read_bytes()
                self.assertEqual(len(payload), entry["bytes"])
                self.assertEqual(hashlib.sha256(payload).hexdigest(), entry["sha256"])


if __name__ == "__main__":
    CMAKE, BASELINE = map(Path, sys.argv[1:3])
    del sys.argv[1:3]
    unittest.main()
