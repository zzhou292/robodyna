"""Check retained FSI/SPH source-language admission and the complete24-demo roster."""

import ast
import hashlib
import json
from pathlib import Path
import shlex
import sys
import unittest

from tools.verification.cmake_evidence import commands


def assignment(path, name):
    for node in ast.parse(path.read_text()).body:
        if isinstance(node, ast.Assign) and node.targets[0].id == name:
            return ast.literal_eval(node.value)
    raise ValueError("Missing source group: " + name)


def cmake_values(path, group):
    result = []
    for row in commands(path.read_text()):
        tokens = shlex.split(row["arguments"])
        if row["command"] == "set" and tokens and tokens[0] == group:
            result.extend(token for token in tokens[1:] if token and not token.startswith("${"))
    return result


class SphSourceInventory(unittest.TestCase):
    def test_exact_generic_and_gpu_source_language_partition(self):
        generic = assignment(FSI_LIST, "FSI_SOURCES")
        self.assertEqual(set(generic), {"src/chrono_fsi/" + path for path in cmake_values(FSI_CMAKE, "FSI_FILES") if path.endswith(".cpp")})
        self.assertEqual(len(generic), 3)
        actual = assignment(SPH_LIST, "SPH_CUDA_SOURCES")
        expected = ["src/chrono_fsi/sph/" + path for path in cmake_values(SPH_CMAKE, "FSISPH_GPU_SOURCE_FILES")]
        self.assertEqual(actual, expected)
        self.assertEqual(len(actual), len(set(actual)))
        self.assertEqual(sum(path.endswith(".cpp") for path in actual), 5)
        self.assertEqual(sum(path.endswith(".cu") for path in actual), 12)
        self.assertFalse(any("LinearSolverBiCGStab.cpp" in path or "LinearSolverGMRES.cpp" in path for path in actual))
        self.assertEqual(assignment(SPH_LIST, "SPH_VISUAL_SOURCES"),
                         ["src/chrono_fsi/sph/" + path for path in cmake_values(SPH_CMAKE, "FSISPH_VSG_FILES") if path.endswith(".cpp")])

    def test_full_sph_roster_and_real_direct_solver_dependencies(self):
        declared = assignment(DEMO_LIST, "SPH_DEMO_SOURCES")
        expected = {"src/demos/fsi/sph/" + name + ".cpp" for name in cmake_values(DEMO_CMAKE, "DEMOS")}
        self.assertEqual(len(declared), 19)
        self.assertEqual(set(declared), expected)
        catalog = json.loads(CATALOG.read_text())
        self.assertEqual((catalog["source_count"], catalog["declared_sph"], catalog["pending_tdpf"]), (24, 19, 5))
        self.assertEqual({row["source"] for row in catalog["sph"]}, expected)
        self.assertEqual(len(assignment(DEMO_LIST, "TDPF_DEMO_SOURCES")), 5)
        self.assertEqual({row["source"] for row in catalog["tdpf"]}, set(assignment(DEMO_LIST, "TDPF_DEMO_SOURCES")))
        self.assertTrue(all(row["target"] is None and row["requires"] for row in catalog["tdpf"]))
        targets = {}
        for node in ast.parse(DEMO_BUILD.read_text()).body:
            if isinstance(node, ast.Expr) and isinstance(node.value, ast.Call) and isinstance(node.value.func, ast.Name):
                if node.value.func.id == "robodyna_cpp_demo":
                    attrs = {item.arg: ast.literal_eval(item.value) for item in node.value.keywords}
                    targets[attrs["name"]] = attrs
        self.assertEqual(set(targets), {row["name"] for row in catalog["sph"]})
        self.assertEqual(sum("pardiso_mkl" in row["features"] for row in catalog["sph"]), 4)
        for row in catalog["sph"]:
            self.assertEqual(targets[row["name"]]["features"], row["features"])
            self.assertEqual(targets[row["name"]]["srcs"], ["//src/compatibility/chrono:" + row["source"]])

    def test_retained_clock_algorithm_header_and_demo_bytes_unchanged(self):
        baseline = json.loads(BASELINE.read_text())
        self.assertEqual(baseline["schema"], "robodyna.retained_fsi_sph_build.v1")
        source_root = FSI_CMAKE.parents[2]
        for record in baseline["files"]:
            path = source_root / record["path"]
            with self.subTest(path=record["path"]):
                self.assertEqual(path.stat().st_size, record["bytes"])
                self.assertEqual(hashlib.sha256(path.read_bytes()).hexdigest(), record["sha256"])


if __name__ == "__main__":
    (FSI_LIST, SPH_LIST, FSI_CMAKE, SPH_CMAKE, BASELINE, DEMO_LIST,
     DEMO_CMAKE, DEMO_BUILD, CATALOG) = [Path(value) for value in sys.argv[1:10]]
    del sys.argv[1:10]
    unittest.main()
