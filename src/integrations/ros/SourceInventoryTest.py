"""Authenticate retained ROS source groups, real demos and explicit prerequisites."""

import ast
import hashlib
import json
from pathlib import Path
import shlex
import sys
import unittest

from tools.verification.cmake_evidence import commands


def groups(path):
    for statement in ast.parse(path.read_text()).body:
        if isinstance(statement, ast.Assign) and statement.targets[0].id == "ROS_GROUPS":
            return ast.literal_eval(statement.value)
    raise ValueError("Expected explicit ROS source groups")


class RosSourceInventory(unittest.TestCase):
    def test_every_original_translation_unit_has_one_declared_family(self):
        declared = groups(SOURCES)
        rows = [path for group in declared.values() for path in group["sources"]]
        cmake_paths = set()
        for row in commands(CMAKE.read_text()):
            if row["command"] in ("set", "list", "add_executable"):
                for token in shlex.split(row["arguments"]):
                    if token.endswith(".cpp"):
                        cmake_paths.add("src/chrono_ros/" + token)
        self.assertEqual(set(rows), cmake_paths)
        self.assertEqual((len(rows), len(set(rows))), (27, 27))
        self.assertEqual(len(declared["protocol"]["sources"]), 7)
        self.assertEqual(len(declared["node"]["sources"]), 2)

    def test_original_bytes_are_unchanged(self):
        manifest = json.loads(BASELINE.read_text())
        root = CMAKE.parents[2]
        for path, digest in manifest["files"].items():
            relative = path.removeprefix("src/compatibility/chrono/")
            with self.subTest(source=relative):
                self.assertEqual(hashlib.sha256((root / relative).read_bytes()).hexdigest(), digest)

    def test_seven_real_mains_and_two_explicit_optional_profiles(self):
        catalog = json.loads(CATALOG.read_text())
        targets = {}
        for statement in ast.parse(DEMO_BUILD.read_text()).body:
            if not isinstance(statement, ast.Expr) or not isinstance(statement.value, ast.Call):
                continue
            call = statement.value
            if isinstance(call.func, ast.Name) and call.func.id in ("robodyna_cpp_demo", "cc_binary"):
                values = {row.arg: ast.literal_eval(row.value) for row in call.keywords
                          if row.arg in ("name", "srcs", "deps", "data")}
                targets[values["name"]] = values
        expected = {row["target"].split(":")[1]: row for row in catalog["programs"] if row["declared"]}
        self.assertEqual(set(targets), set(expected))
        self.assertEqual(len(targets), 7)
        self.assertEqual(len(catalog["programs"]), 7)
        root = CMAKE.parents[2]
        for name, row in expected.items():
            original = row["source"].removeprefix("src/compatibility/chrono/")
            self.assertEqual(targets[name]["srcs"], ["//src/compatibility/chrono:" + original])
            self.assertEqual(hashlib.sha256((root / original).read_bytes()).hexdigest(), row["sha256"])
            self.assertIn("//src/integrations/ros:node", targets[name]["data"])
        for row in catalog["programs"]:
            if not row["declared"]:
                self.assertTrue(row["required_optional_profile"])
        self.assertEqual(expected["sensor"]["required_profiles"], ["ros-sensor", "sensor-optix"])
        self.assertEqual(expected["urdf"]["required_profiles"], ["ros-urdf", "vsg"])


if __name__ == "__main__":
    SOURCES, BASELINE, CMAKE, CATALOG, DEMO_BUILD = [Path(value) for value in sys.argv[1:6]]
    del sys.argv[1:6]
    unittest.main()
