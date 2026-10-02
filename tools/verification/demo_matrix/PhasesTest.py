"""Check phase resource/profile isolation without running compilers."""

import copy
from pathlib import Path
import tempfile
import unittest

from tools.verification.demo_matrix.phases import prerequisite_plan


class PhasesTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name)
        self.demo = {"batch": "python_sph", "phase": "demo_targets", "configs": ["cuda", "sm120", "fsi-sph", "vsg"],
                     "targets": ["//examples/python/fsi:object_drop"], "source_by_target": {"//examples/python/fsi:object_drop": "original.py"},
                     "resources": {"affinity_cpus": 8, "compiler_workers": 2, "rss_gib": 16, "minimum_available_ram_gib": 32},
                     "command": ["bazel", "build", "--jobs=2", "--config=fsi-sph", "--config=vsg",
                                 "--repo_env=ROBODYNA_SDK=/qualified/sdk", "--build_event_json_file=/old/events.jsonl",
                                 "//examples/python/fsi:object_drop"]}

    def tearDown(self):
        self.temporary.cleanup()

    def test_native_phase_preserves_configuration_and_does_not_count_demos(self):
        original = copy.deepcopy(self.demo)
        batch = {"prerequisite_targets": ["//build_defs/bindings:native_core", "//build_defs/bindings/optional:native_fsi"]}
        phase = prerequisite_plan(self.demo, batch, self.root / "native")
        self.assertEqual(self.demo, original)
        self.assertEqual(phase["configs"], original["configs"])
        self.assertIn("--config=fsi-sph", phase["command"])
        self.assertIn("--config=vsg", phase["command"])
        self.assertIn("--repo_env=ROBODYNA_SDK=/qualified/sdk", phase["command"])
        self.assertEqual(phase["resources"]["compiler_workers"], 4)
        self.assertEqual(phase["resources"]["affinity_cpus"], 8)
        self.assertEqual(phase["resources"]["rss_gib"], 16)
        self.assertEqual(phase["source_by_target"], {})
        self.assertNotIn(original["targets"][0], phase["targets"])
        self.assertEqual(phase["phase"], "native_prerequisites")
        self.assertFalse((self.root / "native").exists())

    def test_no_prerequisites_preserves_single_phase_workflow(self):
        self.assertIsNone(prerequisite_plan(self.demo, {}, self.root / "unused"))

    def test_wildcards_duplicates_and_demo_aliasing_are_rejected(self):
        for values in (["//build_defs/bindings:*"] , ["//x:lib", "//x:lib"], self.demo["targets"]):
            with self.subTest(values=values), self.assertRaises(ValueError):
                prerequisite_plan(self.demo, {"prerequisite_targets": values}, self.root / "native")


if __name__ == "__main__":
    unittest.main()
