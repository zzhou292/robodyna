"""Qualify the admitted source partition and preserve the complete demo denominator."""

import ast
import hashlib
import json
import os
from pathlib import Path
import re
import sys
import unittest

from src.vehicle.CMakeSources import PROFILE, library_groups, source_groups


def assignment(path, name):
    for node in ast.parse(path.read_text()).body:
        if isinstance(node, ast.Assign) and node.targets[0].id == name:
            return ast.literal_eval(node.value)
    raise ValueError("Expected explicit source assignment: " + name)


class VehicleSourceInventory(unittest.TestCase):
    def test_recursive_demo_headers_include_source_root_helpers(self):
        source_root = VEHICLE_CMAKE.parents[2].absolute()
        rows = json.loads(CATALOG.read_text())["programs"]
        rules = {}
        for node in ast.parse(DEMO_BUILD.read_text()).body:
            if isinstance(node, ast.Expr) and isinstance(node.value, ast.Call):
                attrs = {keyword.arg: keyword.value for keyword in node.value.keywords}
                if "name" in attrs:
                    rules[ast.literal_eval(attrs["name"])] = attrs
        providers = {
            "src/demos/SetChronoSolver.h": "solver_selection_headers",
            "src/demos/vehicle/WheeledVehicleModels.h": "wheeled_models_headers",
            "src/demos/vehicle/WheeledVehicleJSON.h": "wheeled_json_headers",
        }
        all_headers = set()
        for row in rows:
            discovered = set()
            pending = [source_root / row["source"]]
            while pending:
                path = pending.pop()
                for include in re.findall(r'^\s*#\s*include\s*"([^"]+)"', path.read_text(), re.M):
                    candidates = (path.parent / include, source_root / "src" / include)
                    for candidate in candidates:
                        candidate = Path(os.path.abspath(candidate))
                        if candidate.is_file() and candidate.is_relative_to(source_root / "src/demos"):
                            relative = candidate.relative_to(source_root).as_posix()
                            if relative not in discovered:
                                discovered.add(relative)
                                pending.append(candidate)
                            break
                    else:
                        self.assertFalse(include.startswith("demos/"), "Missing helper: " + include)
            with self.subTest(program=row["name"]):
                self.assertEqual(set(row["local_headers"]), discovered)
                all_headers.update(discovered)
                if row["status"] != "declared_for_compile":
                    continue
                attrs = rules[row["name"]]
                for header in discovered:
                    label = "//src/compatibility/chrono:" + header
                    if header in providers:
                        provider = providers[header]
                        self.assertIn(":" + provider, ast.literal_eval(attrs["deps"]))
                        self.assertIn(label, ast.literal_eval(rules[provider]["hdrs"]))
                        self.assertEqual(ast.literal_eval(rules[provider]["strip_include_prefix"]),
                                         "/src/compatibility/chrono/src")
                    else:
                        self.assertIn(label, ast.literal_eval(attrs["srcs"]))
        self.assertEqual(all_headers, set(assignment(DEMO_LIST, "VEHICLE_DEMO_HEADERS")))

    def test_exact_cmake_partition_reuses_scm_and_stb(self):
        declared = assignment(VEHICLE_LIST, "VEHICLE_GROUPS")
        original = source_groups(VEHICLE_CMAKE, "src/chrono_vehicle", "CV")
        admitted = library_groups(VEHICLE_CMAKE, "Chrono_vehicle", original)
        expected = {path for values in admitted.values() for path in values if path.endswith(".cpp")}
        shared = {"src/chrono_vehicle/" + path for path in (
            "ChTerrain.cpp", "ChWorldFrame.cpp", "ChVehicleDataPath.cpp", "terrain/SCMTerrain.cpp")}
        shared.update("src/chrono_thirdparty/stb/" + path for path in ("stb_image.cpp", "stb_image_write.cpp"))
        actual = [path for group in declared.values() for path in group["sources"]]
        self.assertEqual(len(actual), 229)
        self.assertEqual(len(actual), len(set(actual)))
        self.assertFalse(set(actual) & shared)
        self.assertEqual(set(actual) | shared, expected)
        self.assertNotIn("src/chrono_vehicle/terrain/CRGTerrain.cpp", actual)
        self.assertNotIn("src/chrono_vehicle/terrain/CRMTerrain.cpp", actual)
        render = assignment(VISUAL_LIST, "VISUALIZATION_GROUPS")
        for name, cmake_groups, reused in (
                ("vsg", ("CVVSG_FILES", "CVVSG_WV_FILES", "CVVSG_TV_FILES"),
                 {"src/chrono_vehicle/visualization/ChScmVisualizationVSG.cpp"}),
                ("irrlicht", ("CVIRR_FILES", "CVIRR_WV_FILES", "CVIRR_TV_FILES"), set())):
            selected = {path for key in cmake_groups for path in original[key] if path.endswith(".cpp")}
            self.assertEqual(len(render[name]["sources"]), 5)
            self.assertEqual(set(render[name]["sources"]) | reused, selected)
            self.assertFalse(set(render[name]["sources"]) & reused)

    def test_final_model_families_do_not_repeat_nested_cmake_groups(self):
        declared = assignment(MODEL_LIST, "MODEL_GROUPS")
        original = source_groups(MODEL_CMAKE, "src/chrono_models/vehicle", "CVM")
        admitted = library_groups(MODEL_CMAKE, "ChronoModels_vehicle", original)
        self.assertEqual({group["cmake_group"] for group in declared.values()}, set(admitted))
        actual = [path for group in declared.values() for path in group["sources"]]
        expected = {path for values in admitted.values() for path in values if path.endswith(".cpp")}
        self.assertEqual(len(actual), 389)
        self.assertEqual(len(actual), len(set(actual)))
        self.assertEqual(set(actual), expected)

    def test_all_64_demos_have_explicit_admission_or_pending_requirements(self):
        catalog = json.loads(CATALOG.read_text())
        self.assertEqual(catalog["schema"], "robodyna.vehicle_demo_admission.v1")
        rows = catalog["programs"]
        self.assertEqual(len(rows), 64)
        self.assertEqual(len({row["source"] for row in rows}), 64)
        self.assertEqual({row["source"] for row in rows}, set(assignment(DEMO_LIST, "VEHICLE_DEMO_SOURCES")))
        admitted = [row for row in rows if row["status"] == "declared_for_compile"]
        pending = [row for row in rows if row["status"] == "pending_prerequisites"]
        self.assertEqual((len(admitted), len(pending)), (38, 26))
        self.assertTrue(all(row["target"] and not row["requires"] for row in admitted))
        self.assertTrue(all(row["target"] is None and row["requires"] for row in pending))
        targets = {}
        for node in ast.parse(DEMO_BUILD.read_text()).body:
            if isinstance(node, ast.Expr) and isinstance(node.value, ast.Call) and isinstance(node.value.func, ast.Name):
                if node.value.func.id == "robodyna_cpp_demo":
                    attributes = {keyword.arg: ast.literal_eval(keyword.value) for keyword in node.value.keywords}
                    targets[attributes["name"]] = attributes
        self.assertEqual(set(targets), {row["name"] for row in admitted})
        for row in admitted:
            self.assertIn("//src/compatibility/chrono:" + row["source"], targets[row["name"]]["srcs"])
            profile = {
                "paths": ["vehicle", "postprocess"],
                "scm_terrain_fea_tire": ["vehicle", "vsg", "pardiso_mkl"],
                "track_test_rig_band": ["vehicle", "vsg", "pardiso_mkl"],
            }.get(row["name"], ["vehicle", "vsg", "irrlicht"])
            self.assertEqual(targets[row["name"]]["features"], profile)

    def test_preintegration_source_bytes_and_profile_are_preserved(self):
        baseline = json.loads(BASELINE.read_text())
        self.assertEqual(baseline["schema"], "robodyna.retained_vehicle_build.v1")
        self.assertEqual(baseline["profile"], PROFILE)
        source_root = VEHICLE_CMAKE.parents[2]
        for entry in baseline["files"]:
            path = source_root / entry["path"]
            with self.subTest(path=entry["path"]):
                self.assertEqual(path.stat().st_size, entry["bytes"])
                self.assertEqual(hashlib.sha256(path.read_bytes()).hexdigest(), entry["sha256"])


if __name__ == "__main__":
    (VEHICLE_LIST, MODEL_LIST, VISUAL_LIST, VEHICLE_CMAKE, MODEL_CMAKE,
     BASELINE, CATALOG, DEMO_LIST, DEMO_BUILD) = [Path(value) for value in sys.argv[1:10]]
    del sys.argv[1:10]
    unittest.main()
