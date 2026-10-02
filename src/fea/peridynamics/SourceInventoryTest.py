"""Authenticate the retained module and include its unlisted demo entry points."""

import ast
import hashlib
import json
from pathlib import Path
import shlex
import sys
import unittest

from tools.verification.cmake_evidence import commands
from tools.migration.source_transform import index_entries, original_bytes


class SourceInventory(unittest.TestCase):
    def test_module_matches_owning_cmake(self):
        native = {}
        for statement in ast.parse(SOURCES.read_text()).body:
            if isinstance(statement, ast.Assign) and isinstance(statement.value, ast.List):
                native[statement.targets[0].id] = ast.literal_eval(statement.value)
        groups = {}
        for command in commands(CMAKE.read_text()):
            if command["command"] == "set":
                tokens = shlex.split(command["arguments"])
                if tokens:
                    groups[tokens[0]] = tokens[1:]
        for kind in ("SOURCES", "HEADERS"):
            actual = native["PERIDYNAMICS_" + kind]
            expected = ["src/chrono_peridynamics/" + path for path in groups["Chrono_PERIDYNAMICS_" + kind]]
            self.assertEqual(len(actual), len(set(actual)))
            self.assertEqual(set(actual), set(expected))

    def test_only_recorded_api_adaptations_changed(self):
        root = CMAKE.parents[2]
        baseline = json.loads(BASELINE.read_text())
        transformations = index_entries(json.loads(LEDGER.read_text()))
        for row in baseline["files"]:
            with self.subTest(path=row["path"]):
                entry = transformations.get("src/compatibility/chrono/" + row["path"])
                data = original_bytes(root.parents[2], entry) if entry else (root / row["path"]).read_bytes()
                self.assertEqual(hashlib.sha256(data).hexdigest(), row["sha256"])

    def test_experimental_programs_remain_explicit(self):
        text = BUILD.read_text()
        self.assertIn('["elastic", "fracture", "implicit", "benchmark", "fluid"]', text)
        source = (CMAKE.parents[2] / "src/demos/peridynamics/demo_PERI_fluid.cpp").read_text()
        # Do not claim that exposing this inherited disabled program qualifies it.
        self.assertIn("assert(false)", source)
        self.assertLess(source.index("return 0;"), source.index("ChSystemNSC mphysicalSystem;"))


if __name__ == "__main__":
    SOURCES, BASELINE, CMAKE, BUILD, LEDGER = [Path(value) for value in sys.argv[1:6]]
    del sys.argv[1:6]
    unittest.main()
