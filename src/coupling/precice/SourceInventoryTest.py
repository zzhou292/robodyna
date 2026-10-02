"""Authenticate the retained preCICE module and all its executable entry points."""

import ast
import hashlib
import json
from pathlib import Path
import shlex
import sys
import unittest

from tools.verification.cmake_evidence import commands


class SourceInventoryTest(unittest.TestCase):
    def test_three_original_cpp_owners_match_cmake(self):
        groups = None
        for statement in ast.parse(SOURCES.read_text()).body:
            if isinstance(statement, ast.Assign) and statement.targets[0].id == "PRECICE_GROUPS":
                groups = ast.literal_eval(statement.value)
        self.assertIsNotNone(groups)
        actual = [p for g in groups.values() for p in g["sources"]]
        expected = set()
        for row in commands(CMAKE.read_text()):
            if row["command"] == "set":
                expected.update("src/chrono_precice/" + value for value in shlex.split(row["arguments"]) if value.endswith(".cpp"))
        self.assertEqual(set(actual), expected)
        self.assertEqual(len(actual), 3)

    def test_preserved_source_and_asset_bytes(self):
        for path, digest in json.loads(BASELINE.read_text())["files"].items():
            with self.subTest(path=path):
                self.assertEqual(hashlib.sha256((CMAKE.parents[2] / path).read_bytes()).hexdigest(), digest)

    def test_both_demos_and_non_demo_yaml_application_are_recorded(self):
        programs = json.loads(CATALOG.read_text())["programs"]
        self.assertEqual(len(programs), 3)
        self.assertEqual({row["target"] for row in programs}, {
            "//examples/coupling/precice:sphere_drop", "//examples/coupling/precice:flap_openfoam", "//apps/precice:run_adapter"})
        for row in programs:
            self.assertIn("yaml", row["required_profiles"])
            path = row["source"].removeprefix("src/compatibility/chrono/")
            self.assertEqual(hashlib.sha256((CMAKE.parents[2] / path).read_bytes()).hexdigest(), row["sha256"])
            self.assertIn("int main(", (CMAKE.parents[2] / path).read_text())
            if row["target"].endswith(":sphere_drop"):
                self.assertIn("fsi-sph", row["required_profiles"])


if __name__ == "__main__":
    SOURCES, BASELINE, CMAKE, CATALOG = [Path(v) for v in sys.argv[1:5]]
    del sys.argv[1:5]
    unittest.main()
