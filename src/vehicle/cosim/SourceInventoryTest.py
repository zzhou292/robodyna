"""Authenticate real co-simulation families and original MPI entry points."""

import ast
import hashlib
import json
from pathlib import Path
import shlex
import sys
import unittest

from tools.verification.cmake_evidence import commands


def source_groups(path):
    for statement in ast.parse(path.read_text()).body:
        if isinstance(statement, ast.Assign) and statement.targets[0].id == "COSIM_GROUPS":
            return ast.literal_eval(statement.value)
    raise ValueError("Missing explicit co-simulation source families")


class SourceInventoryTest(unittest.TestCase):
    def test_complete_original_cmake_source_set(self):
        rows = [p for g in source_groups(SOURCES).values() for p in g["sources"]]
        expected = set()
        for row in commands(CMAKE.read_text()):
            if row["command"] == "set":
                for token in shlex.split(row["arguments"]):
                    if token.endswith(".cpp"):
                        expected.add("src/chrono_vehicle/cosim/" + token)
        self.assertEqual(set(rows), expected)
        self.assertEqual((len(rows), len(set(rows))), (20, 20))

    def test_all_pinned_source_bytes_are_unchanged(self):
        root = CMAKE.parents[3]
        for path, digest in json.loads(BASELINE.read_text())["files"].items():
            with self.subTest(path=path):
                self.assertEqual(hashlib.sha256((root / path).read_bytes()).hexdigest(), digest)

    def test_eight_original_mains_with_honest_profiles(self):
        declarations = {}
        for statement in ast.parse(DEMO_BUILD.read_text()).body:
            if not isinstance(statement, ast.Expr) or not isinstance(statement.value, ast.Call):
                continue
            call = statement.value
            if isinstance(call.func, ast.Name) and call.func.id == "robodyna_cpp_demo":
                values = {v.arg: ast.literal_eval(v.value) for v in call.keywords}
                declarations[values["name"]] = values
        catalog = json.loads(CATALOG.read_text())["programs"]
        self.assertEqual(len(catalog), 8)
        self.assertEqual(set(declarations), {r["target"].split(":")[1] for r in catalog})
        for row in catalog:
            actual = declarations[row["target"].split(":")[1]]
            path = row["source"].removeprefix("src/compatibility/chrono/")
            self.assertEqual(actual["srcs"], ["//src/compatibility/chrono:" + path])
            self.assertEqual(hashlib.sha256((CMAKE.parents[3] / path).read_bytes()).hexdigest(), row["sha256"])
            self.assertNotIn("vsg", actual.get("features", []))
            self.assertNotIn("postprocess", actual.get("features", []))
            if row["required_profiles"] == ["multicore"]:
                self.assertIn("//src/vehicle/cosim:omp_terrain", actual["deps"])
            if row["required_profiles"] == ["fsi-sph"]:
                self.assertIn("//src/vehicle/cosim:sph_terrain", actual["deps"])


if __name__ == "__main__":
    SOURCES, BASELINE, CMAKE, CATALOG, DEMO_BUILD = [Path(v) for v in sys.argv[1:6]]
    del sys.argv[1:6]
    unittest.main()
