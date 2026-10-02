"""Authenticate native Multicore source ownership without configuring a build."""

import ast
import hashlib
import json
from pathlib import Path
import shlex
import sys
import unittest

from tools.verification.cmake_evidence import commands


def groups(path):
    result = {}
    for command in commands(path.read_text()):
        if command["command"] == "set":
            words = shlex.split(command["arguments"])
            if words:
                result.setdefault(words[0], set()).update(words[1:])
    return result


def literals(path):
    result = {}
    for node in ast.parse(path.read_text()).body:
        if isinstance(node, ast.Assign):
            try:
                result[node.targets[0].id] = ast.literal_eval(node.value)
            except ValueError:
                pass  # Only derived export concatenation is omitted.
    return result


class SourceInventoryTest(unittest.TestCase):
    def test_exact_core_and_module_translation_units(self):
        native = literals(SOURCES)
        module = groups(MODULE_CMAKE)
        core = groups(CORE_CMAKE)
        core_paths = {"src/chrono/" + value for key, values in core.items()
                      if key.startswith(("Chrono_MulticoreMath_", "Chrono_collision_multicore_"))
                      for value in values if value.endswith((".cpp", ".h"))}
        module_paths = {"src/chrono_multicore/" + value for key, values in module.items()
                        if key in ("Chrono_Multicore_BASE", "Chrono_Multicore_PHYSICS",
                                   "Chrono_Multicore_SOLVER", "Chrono_Multicore_CONSTRAINTS",
                                   "Chrono_Multicore_COLLISION")
                        for value in values if value.endswith((".cpp", ".h"))}
        for name, expected, count in (("CORE_EXTENSION", core_paths, 15),
                                      ("MULTICORE", module_paths, 27)):
            self.assertEqual(len(native[name + "_SOURCES"]), count)
            self.assertEqual(set(native[name + "_SOURCES"]),
                             {path for path in expected if path.endswith(".cpp")})
            self.assertEqual(set(native[name + "_HEADERS"]),
                             {path for path in expected if path.endswith(".h")})
        self.assertFalse(set(native["CORE_EXTENSION_SOURCES"]) & set(native["MULTICORE_SOURCES"]))

    def test_all_fifteen_original_entrypoints_are_retained(self):
        expected = {"src/demos/multicore/" + value + ".cpp"
                    for value in groups(DEMO_CMAKE)["DEMOS"] if value.startswith("demo_")}
        self.assertEqual(len(expected), 15)
        self.assertEqual(set(literals(SOURCES)["MULTICORE_DEMO_SOURCES"]), expected)

    def test_existing_core_and_chomp_compile_owners_remain_unique(self):
        native = literals(SOURCES)
        baseline = literals(BASELINE_OWNERS)["NATIVE_SOURCE_GROUPS"]
        existing = [path for group in baseline.values() for path in group]
        self.assertEqual(existing.count("src/chrono/utils/ChOpenMP.cpp"), 1)
        additions = native["CORE_EXTENSION_SOURCES"] + native["MULTICORE_SOURCES"]
        self.assertFalse(set(existing) & set(additions))

    def test_retained_source_and_asset_bytes(self):
        baseline = json.loads(BASELINE.read_text())
        self.assertEqual(baseline["schema"], "robodyna.retained_multicore_build.v1")
        source_root = CORE_CMAKE.parents[2]
        for row in baseline["files"]:
            path = source_root / row["path"]
            with self.subTest(path=row["path"]):
                self.assertEqual(path.stat().st_size, row["bytes"])
                digest = hashlib.sha256()
                with path.open("rb") as stream:
                    for chunk in iter(lambda: stream.read(1 << 20), b""):
                        digest.update(chunk)
                self.assertEqual(digest.hexdigest(), row["sha256"])


if __name__ == "__main__":
    SOURCES, BASELINE, CORE_CMAKE, MODULE_CMAKE, DEMO_CMAKE, BASELINE_OWNERS = [Path(value) for value in sys.argv[1:7]]
    del sys.argv[1:7]
    unittest.main()
