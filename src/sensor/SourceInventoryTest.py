"""Verify retained Vulkan source/profile boundaries and all 16 Sensor demos."""

import ast
import hashlib
import json
from pathlib import Path
import re
import shlex
import sys
import unittest

from src.sensor.CMakeProfile import PROFILE, condition, source_groups
from tools.verification.cmake_evidence import commands


def assignment(path, name):
    for node in ast.parse(path.read_text()).body:
        if isinstance(node, ast.Assign) and node.targets[0].id == name:
            return ast.literal_eval(node.value)
    raise ValueError("Missing explicit assignment: " + name)


class SensorSourceInventory(unittest.TestCase):
    def test_exact_cmake_profile_selects_cpp_and_real_vulkan_shaders(self):
        groups = source_groups(SENSOR_CMAKE)
        declared = assignment(SOURCE_LIST, "SENSOR_GROUPS")
        all_sources = []
        for group in declared.values():
            self.assertEqual(group["sources"], groups[group["cmake_group"] + "_SOURCES"])
            self.assertEqual(group["headers"], groups[group["cmake_group"] + "_HEADERS"])
            all_sources.extend(group["sources"])
        self.assertEqual((len(all_sources), len(set(all_sources))), (54, 54))
        self.assertTrue(all(path.endswith(".cpp") for path in all_sources))
        self.assertFalse(groups["Chrono_sensor_OPTIX_SOURCES"])
        self.assertFalse(groups["Chrono_sensor_METAL_SOURCES"])
        self.assertFalse(groups["Chrono_sensor_CUDA_SOURCES"])
        shaders = assignment(SOURCE_LIST, "SENSOR_SHADERS")
        self.assertEqual(shaders, groups["Chrono_sensor_VULKAN_SHADER_SOURCES"])
        self.assertEqual(len(shaders), 5)
        self.assertTrue(condition("CH_USE_SENSOR_VULKAN_RT AND NOT CH_USE_SENSOR_OPTIX"))
        self.assertFalse(condition("CH_USE_SENSOR_OPTIX OR CH_USE_SENSOR_METAL_RT"))
        with self.assertRaisesRegex(ValueError, "Unreviewed"):
            condition("CH_NEW_UNREVIEWED_BACKEND")

    def test_complete_roster_retains_backend_specific_exclusions(self):
        root = SENSOR_CMAKE.parents[2]
        selected, complete = set(), set()
        profile = dict(PROFILE, CH_ENABLE_MODULE_IRRLICHT=True, CH_ENABLE_MODULE_VEHICLE=True,
                       CH_ENABLE_MODULE_VEHICLE_MODELS=True)
        for row in commands((root / "src/demos/sensor/CMakeLists.txt").read_text()):
            tokens = shlex.split(row["arguments"])
            if row["command"] != "set" or not tokens or tokens[0] != "DEMOS":
                continue
            demos = {token for token in tokens[1:] if token.startswith("demo_SEN_")}
            complete.update(demos)
            if all([condition(expression, profile) for expression in row["conditions"]]):
                selected.update(demos)
        catalog = json.loads(CATALOG.read_text())
        rows = catalog["programs"]
        self.assertEqual(len(rows), 16)
        self.assertEqual({Path(row["source"]).stem for row in rows}, complete)
        admitted = [row for row in rows if row["target"]]
        reviewed = {"gator", "deformablesoil", "metal_quarterpanel"}
        self.assertEqual(set(catalog["reviewed_cmake_gate_extensions"]), reviewed)
        self.assertEqual(len(admitted), 14)
        self.assertEqual({Path(row["source"]).stem for row in admitted if row["name"] not in reviewed}, selected)
        targets = {}
        for node in ast.parse(DEMO_BUILD.read_text()).body:
            if isinstance(node, ast.Expr) and isinstance(node.value, ast.Call) and isinstance(node.value.func, ast.Name):
                if node.value.func.id == "robodyna_cpp_demo":
                    attributes = {keyword.arg: ast.literal_eval(keyword.value) for keyword in node.value.keywords}
                    targets[attributes["name"]] = attributes
        self.assertEqual(set(targets), {row["name"] for row in admitted})
        for row in rows:
            if row["target"]:
                expected_features = ["sensor", "vehicle", "irrlicht"] if row["name"] in reviewed else ["sensor"]
                self.assertEqual(targets[row["name"]]["features"], expected_features)
                self.assertEqual(targets[row["name"]]["deps"], row["deps"])
                self.assertEqual(targets[row["name"]]["srcs"], ["//src/compatibility/chrono:" + row["source"]])
                self.assertFalse(row["requires"])
            else:
                self.assertTrue(row["requires"])
                self.assertFalse(row["compiled"])

    def test_demo_utility_headers_are_declared_beyond_the_cmake_module_list(self):
        root = SENSOR_CMAKE.parents[2]
        groups = assignment(SOURCE_LIST, "SENSOR_GROUPS")
        declared = {path for group in groups.values() for path in group["headers"]}
        declared.update(assignment(SOURCE_LIST, "SENSOR_PRIVATE_HEADERS"))
        for row in json.loads(CATALOG.read_text())["programs"]:
            if not row["target"]:
                continue
            includes = re.findall(r'^\s*#\s*include\s*"(chrono_sensor/[^\"]+)"',
                                  (root / row["source"]).read_text(), re.M)
            for include in includes:
                # Cross-renderer comparison includes these only in the disabled
                # OptiX branch; the real Vulkan utility headers must be present.
                if include.startswith("chrono_sensor/optix/") or include.endswith("ChOptixSensor.h"):
                    continue
                if include == "chrono_sensor/ChConfigSensor.h":
                    continue  # This one is generated from its pinned template.
                self.assertIn("src/" + include, declared, row["name"] + ": " + include)

    def test_retained_source_bytes_are_unchanged(self):
        baseline = json.loads(BASELINE.read_text())
        self.assertEqual(baseline["schema"], "robodyna.retained_sensor_build.v1")
        self.assertEqual(baseline["profile"], PROFILE)
        root = SENSOR_CMAKE.parents[2]
        for entry in baseline["files"]:
            with self.subTest(path=entry["path"]):
                payload = (root / entry["path"]).read_bytes()
                self.assertEqual(len(payload), entry["bytes"])
                self.assertEqual(hashlib.sha256(payload).hexdigest(), entry["sha256"])


if __name__ == "__main__":
    SOURCE_LIST, SENSOR_CMAKE, BASELINE, CATALOG, DEMO_BUILD = [Path(value) for value in sys.argv[1:6]]
    del sys.argv[1:6]
    unittest.main()
