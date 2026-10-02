"""Compare the new native ownership list with retained CMake/source evidence."""

import ast
import hashlib
import json
from pathlib import Path
import posixpath
import shlex
import sys
import unittest

from tools.verification.cmake_evidence import commands


def literal_groups(path):
    result = {}
    for node in ast.parse(path.read_text()).body:
        if isinstance(node, ast.Assign) and len(node.targets) == 1:
            try:
                result[node.targets[0].id] = ast.literal_eval(node.value)
            except ValueError:
                pass  # Derived export concatenation is not a compilation group.
    return result


def cmake_groups(path):
    result = {}
    for command in commands(path.read_text()):
        if command["command"] != "set":
            continue
        tokens = shlex.split(command["arguments"])
        if tokens:
            # Keep declarations from the enabled and disabled VSG branches
            # without evaluating configuration or pretending a build occurred.
            result.setdefault(tokens[0], set()).update(token for token in tokens[1:] if token)
    return result


class DemSourceInventory(unittest.TestCase):
    def test_exact_host_cuda_and_visual_translation_units(self):
        native = literal_groups(SOURCES)
        cmake = cmake_groups(CMAKE)
        def paths(group, suffix):
            return {posixpath.normpath("src/chrono_dem/" + path)
                    for path in cmake[group] if path.endswith(suffix)}
        for group, reference, suffix, count in (
                ("DEM_HOST_SOURCES", "DEM_PHYSICS_FILES", ".cpp", 3),
                ("DEM_CUDA_SOURCES", "DEM_GPU_FILES", ".cu", 2),
                ("DEM_VISUAL_SOURCES", "DEM_VSG_FILES", ".cpp", 1)):
            actual = native[group]
            self.assertEqual(len(actual), count)
            self.assertEqual(len(actual), len(set(actual)))
            self.assertEqual(set(actual), paths(reference, suffix))
        expected_headers = set()
        for name in ("DEM_BASE_FILES", "DEM_PHYSICS_FILES", "DEM_GPU_FILES", "DEM_UTILITY_FILES"):
            expected_headers.update(paths(name, (".h", ".cuh")))
        # These two unchanged shared headers already belong to the core library.
        expected_headers -= {"src/chrono/gpu/ChGpuRuntime.h", "src/chrono/gpu/ChGpuPrimitives.h"}
        self.assertEqual(set(native["DEM_HEADERS"]), expected_headers)
        self.assertEqual(set(native["DEM_VISUAL_HEADERS"]), paths("DEM_VSG_FILES", ".h"))

    def test_all_five_original_demo_entry_points_are_present(self):
        native = literal_groups(SOURCES)
        names = cmake_groups(DEMO_CMAKE)["DEMOS"]
        expected = {"src/demos/dem/" + name + ".cpp" for name in names}
        self.assertEqual(len(expected), 5)
        self.assertEqual(set(native["DEM_DEMO_SOURCES"]), expected)

    def test_retained_source_and_asset_bytes_match_preintegration_snapshot(self):
        baseline = json.loads(BASELINE.read_text())
        self.assertEqual(baseline["schema"], "robodyna.retained_dem_build.v1")
        source_root = CMAKE.parents[2]
        for entry in baseline["files"]:
            path = source_root / entry["path"]
            with self.subTest(path=entry["path"]):
                self.assertEqual(path.stat().st_size, entry["bytes"])
                digest = hashlib.sha256()
                with path.open("rb") as stream:
                    for chunk in iter(lambda: stream.read(1 << 20), b""):
                        digest.update(chunk)
                self.assertEqual(digest.hexdigest(), entry["sha256"])
        template = source_root / literal_groups(SOURCES)["DEM_CONFIG_TEMPLATE"]
        self.assertNotIn("@", template.read_text(), "New template substitutions require explicit configuration review")


if __name__ == "__main__":
    SOURCES, BASELINE, CMAKE, DEMO_CMAKE = [Path(value) for value in sys.argv[1:5]]
    del sys.argv[1:5]
    unittest.main()
