"""Check retained distributed sources, schemas and actual demo entry points."""

import ast
import hashlib
import json
from pathlib import Path
import shlex
import sys
import unittest

from tools.verification.cmake_evidence import commands


class SourceInventoryTest(unittest.TestCase):
    def test_all_original_cpp_and_generated_cxx_are_owned(self):
        actual = None
        for statement in ast.parse(SOURCES.read_text()).body:
            if isinstance(statement, ast.Assign) and statement.targets[0].id == "DISTRIBUTED_SOURCES":
                actual = ast.literal_eval(statement.value)
        self.assertIsNotNone(actual)
        expected = set()
        for row in commands(CMAKE.read_text()):
            if row["command"] in ("set", "list"):
                expected.update("src/chrono_synchrono/" + token for token in shlex.split(row["arguments"])
                                if token.endswith((".cpp", ".cxx")))
        self.assertEqual(set(actual), expected)
        self.assertEqual((len(actual), len(set(actual))), (34, 34))

    def test_original_source_and_schema_bytes(self):
        for path, digest in json.loads(BASELINE.read_text())["files"].items():
            with self.subTest(path=path):
                self.assertEqual(hashlib.sha256((CMAKE.parents[2] / path).read_bytes()).hexdigest(), digest)

    def test_six_mpi_and_three_dds_original_programs(self):
        catalog = json.loads(CATALOG.read_text())["programs"]
        self.assertEqual(len(catalog), 9)
        self.assertEqual(sum(r["transport"] == "mpi" for r in catalog), 6)
        for row in catalog:
            path = CMAKE.parents[2] / row["source"]
            self.assertEqual(hashlib.sha256(path.read_bytes()).hexdigest(), row["sha256"])
            self.assertIn("int main(", path.read_text())
            self.assertTrue(row["target"].startswith("//examples/distributed:"))


if __name__ == "__main__":
    SOURCES, BASELINE, CMAKE, CATALOG = [Path(v) for v in sys.argv[1:5]]
    del sys.argv[1:5]
    unittest.main()
