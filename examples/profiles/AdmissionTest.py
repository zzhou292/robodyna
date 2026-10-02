"""Keep explicit profile batches complete without treating skipped targets as passes."""

import ast
import json
from pathlib import Path
import sys
import unittest


class ProfileAdmission(unittest.TestCase):
    def test_every_profile_entry_names_its_actual_original_main(self):
        rows = json.loads(MATRIX.read_text())["programs"]
        self.assertEqual(len(rows), 21)
        self.assertEqual(len({row["source"] for row in rows}), len(rows))
        root = MATRIX.parents[2]
        known = {row["source"] for path in CATALOGS for row in json.loads(path.read_text())["programs"]}
        for row in rows:
            with self.subTest(target=row["target"]):
                self.assertIn(row["source"], known)
                package, name = row["target"].removeprefix("//").split(":")
                targets = {}
                for node in ast.parse((root / package / "BUILD.bazel").read_text()).body:
                    if isinstance(node, ast.Expr) and isinstance(node.value, ast.Call) and getattr(node.value.func, "id", "") == "robodyna_cpp_demo":
                        attrs = {keyword.arg: ast.literal_eval(keyword.value) for keyword in node.value.keywords}
                        targets[attrs["name"]] = attrs
                self.assertIn(name, targets)
                self.assertIn("//src/compatibility/chrono:" + row["source"], targets[name]["srcs"])
                self.assertFalse(row["compiled"])
                self.assertFalse(row["runtime_qualified"])

    def test_remaining_robot_and_sensor_sources_have_explicit_profile_routes(self):
        rows = json.loads(MATRIX.read_text())["programs"]
        routed = {row["source"] for row in rows}
        for path in CATALOGS[1:]:
            for row in json.loads(path.read_text())["programs"]:
                if not row["target"]:
                    self.assertIn(row["source"], routed)
        by_family = {}
        for row in rows:
            by_family.setdefault(row["profile_family"], []).append(row)
        self.assertEqual({key: len(value) for key, value in by_family.items()},
                         {"crm": 5, "opencrg": 3, "multicore": 6, "robot_crm": 3, "sensor_vulkan": 2, "sensor_optix": 2})
        for row in rows:
            if row["profile_family"] in ("crm", "robot_crm"):
                self.assertEqual(row["required_configs"], ["fsi-sph"])
            elif row["profile_family"] in ("opencrg", "multicore"):
                self.assertEqual(row["required_configs"], [row["profile_family"]])
        crm = next(row for row in rows if row["target"].endswith(":crm_rendering"))
        self.assertEqual(crm["required_configs"], ["sensor-optix-sph"])


if __name__ == "__main__":
    MATRIX, *CATALOGS = [Path(value) for value in sys.argv[1:5]]
    del sys.argv[1:5]
    unittest.main()
