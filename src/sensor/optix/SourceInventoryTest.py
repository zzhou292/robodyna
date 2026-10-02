"""Prove retained OptiX source, language and runtime shader-input partitions."""

import ast
import hashlib
import json
from pathlib import Path
import re
import sys
import unittest

from src.sensor.CMakeProfile import source_groups


def assignments(path):
    result = {}
    for node in ast.parse(path.read_text()).body:
        if isinstance(node, ast.Assign):
            result[node.targets[0].id] = ast.literal_eval(node.value)
    return result


class OptixSourceInventory(unittest.TestCase):
    def test_cmake_partition_keeps_host_cuda_and_runtime_compilation_distinct(self):
        declared = assignments(LIST)
        groups = source_groups(CMAKE, declared["OPTIX_PROFILE"])
        host = []
        for group in declared["OPTIX_HOST_GROUPS"].values():
            self.assertEqual(group["sources"], groups[group["cmake_group"] + "_SOURCES"])
            self.assertEqual(group["headers"], groups[group["cmake_group"] + "_HEADERS"])
            host.extend(group["sources"])
        cuda = declared["OPTIX_CUDA_SOURCES"] + declared["OPTIX_SPH_CUDA_SOURCES"]
        self.assertEqual((len(host), len(set(host))), (55, 55))
        self.assertEqual((len(cuda), len(set(cuda))), (13, 13))
        self.assertEqual(set(cuda), set(groups["Chrono_sensor_CUDA_SOURCES"]))
        self.assertEqual(declared["OPTIX_RUNTIME_SHADERS"], groups["Chrono_sensor_RT_SOURCES"])
        self.assertEqual(len(declared["OPTIX_RUNTIME_SHADERS"]), 12)
        self.assertFalse(set(cuda) & set(declared["OPTIX_RUNTIME_SHADERS"]))
        self.assertFalse(groups["Chrono_sensor_VULKAN_SOURCES"])
        self.assertFalse(groups["Chrono_sensor_METAL_SOURCES"])

    def test_runtime_project_includes_have_exact_declared_inputs(self):
        declared = assignments(LIST)
        shaders = declared["OPTIX_RUNTIME_SHADERS"]
        headers = set(declared["OPTIX_RUNTIME_HEADERS"])
        root = CMAKE.parents[2]
        for source in shaders + list(headers):
            for include in re.findall(r'^\s*#\s*include\s*"([^"]+)"', (root / source).read_text(), re.M):
                if include.startswith("chrono/") or include.startswith("chrono_sensor/"):
                    self.assertIn("src/" + include, headers, source + ": " + include)

    def test_no_original_numerical_or_shader_source_has_changed(self):
        baseline = json.loads(BASELINE.read_text())
        self.assertEqual(baseline["profile"], assignments(LIST)["OPTIX_PROFILE"])
        root = CMAKE.parents[2]
        for entry in baseline["files"]:
            with self.subTest(path=entry["path"]):
                payload = (root / entry["path"]).read_bytes()
                self.assertEqual(len(payload), entry["bytes"])
                self.assertEqual(hashlib.sha256(payload).hexdigest(), entry["sha256"])


if __name__ == "__main__":
    LIST, CMAKE, BASELINE = [Path(value) for value in sys.argv[1:4]]
    del sys.argv[1:4]
    unittest.main()
