"""Check CMake ownership, complete robot admission and unchanged input bytes."""

import ast
import hashlib
import json
import os
from pathlib import Path
import posixpath
import re
import shlex
import sys
import unittest

from tools.verification.cmake_evidence import commands


def assignment(path, name):
    for node in ast.parse(path.read_text()).body:
        if isinstance(node, ast.Assign) and node.targets[0].id == name:
            return ast.literal_eval(node.value)
    raise ValueError("Expected explicit source assignment: " + name)


class RobotSourceInventory(unittest.TestCase):
    def test_cmake_model_groups_have_one_owner_and_urdf_is_explicitly_pending(self):
        declared = assignment(MODEL_LIST, "ROBOT_MODEL_GROUPS")
        expected = {}
        optional = []
        final_groups = []
        for row in commands(MODEL_CMAKE.read_text()):
            tokens = shlex.split(row["arguments"])
            if row["command"] == "add_library" and tokens[0] == "ChronoModels_robot":
                final_groups = [value[2:-1] for value in tokens[1:]]
            if row["command"] != "set" or not tokens or not tokens[0].startswith("CRM_"):
                continue
            if row["conditions"]:
                self.assertEqual(row["conditions"], ["CH_ENABLE_MODULE_PARSERS AND CH_USE_URDF"])
                self.assertEqual(tokens[1], "${CRM_ROBOSIMIAN_FILES}")
                optional.extend(posixpath.normpath("src/chrono_models/robot/" + path) for path in tokens[2:])
            else:
                expected[tokens[0]] = [posixpath.normpath("src/chrono_models/robot/" + path) for path in tokens[1:]]
        self.assertEqual(set(final_groups), set(expected))
        self.assertEqual({group["cmake_group"] for group in declared.values()}, set(expected))
        sources = []
        for group in declared.values():
            self.assertEqual(set(group["sources"] + group["headers"]), set(expected[group["cmake_group"]]))
            sources.extend(group["sources"])
        self.assertEqual((len(sources), len(set(sources))), (16, 16))
        self.assertEqual(set(optional), set(assignment(MODEL_LIST, "ROBOT_PENDING_URDF")))
        self.assertFalse(set(sources) & set(optional))

    def test_every_cmake_demo_has_an_honest_target_or_prerequisite(self):
        source_root = MODEL_CMAKE.parents[3].absolute()
        catalog = json.loads(CATALOG.read_text())
        rows = catalog["programs"]
        self.assertEqual(catalog["schema"], "robodyna.robotics_demo_admission.v1")
        expected_mains = set()
        for cmake in assignment(DEMO_LIST, "ROBOT_DEMO_CMAKE_FILES"):
            for command in commands((source_root / cmake).read_text()):
                for token in shlex.split(command["arguments"]):
                    if re.fullmatch(r"demo_ROBOT_[A-Za-z0-9_]+", token):
                        expected_mains.add((Path(cmake).parent / (token + ".cpp")).as_posix())
        self.assertEqual(len(expected_mains), 17)
        self.assertEqual({row["source"] for row in rows}, expected_mains)
        self.assertEqual(set(assignment(DEMO_LIST, "ROBOT_DEMO_SOURCES")), expected_mains)
        self.assertEqual(sum(row["status"] == "declared_for_compile" for row in rows), 11)
        self.assertEqual(sum(row["status"] == "pending_prerequisites" for row in rows), 6)
        targets = {}
        for node in ast.parse(DEMO_BUILD.read_text()).body:
            if isinstance(node, ast.Expr) and isinstance(node.value, ast.Call) and isinstance(node.value.func, ast.Name):
                if node.value.func.id == "robodyna_cpp_demo":
                    attrs = {keyword.arg: ast.literal_eval(keyword.value) for keyword in node.value.keywords}
                    targets[attrs["name"]] = attrs
        self.assertEqual(set(targets), {row["name"] for row in rows if row["target"]})
        for row in rows:
            with self.subTest(program=row["name"]):
                if row["target"]:
                    target = targets[row["name"]]
                    self.assertIn("//src/compatibility/chrono:" + row["source"], target["srcs"])
                    self.assertEqual(target["deps"], row["deps"])
                    self.assertEqual(target["features"], row["features"])
                    self.assertFalse(row["requires"])
                    self.assertNotIn("vehicle_crm", row["features"])
                else:
                    self.assertTrue(row["requires"])
                    self.assertFalse(row["features"])

    def test_local_headers_resolve_both_relative_and_source_root_includes(self):
        root = MODEL_CMAKE.parents[3].absolute()
        all_headers = set()
        for row in json.loads(CATALOG.read_text())["programs"]:
            seen = set()
            pending = [root / row["source"]]
            while pending:
                source = pending.pop()
                for include in re.findall(r'^\s*#\s*include\s*"([^"]+)"', source.read_text(), re.M):
                    for candidate in (source.parent / include, root / "src" / include):
                        candidate = Path(os.path.abspath(candidate))
                        if candidate.is_file() and candidate.is_relative_to(root / "src/demos"):
                            relative = candidate.relative_to(root).as_posix()
                            if relative not in seen:
                                seen.add(relative)
                                pending.append(candidate)
                            break
                    else:
                        self.assertFalse(include.startswith("demos/"), "Missing helper: " + include)
            self.assertEqual(set(row["local_headers"]), seen)
            all_headers.update(seen)
            if row["target"] and seen:
                self.assertEqual(seen, {"src/demos/SetChronoSolver.h", "src/demos/robot/lander/model/Lander.h"})
                self.assertIn(":lander_model", row["deps"])
                self.assertIn("//examples/vehicle:solver_selection_headers", row["deps"])
        self.assertEqual(set(assignment(DEMO_LIST, "ROBOT_DEMO_HEADERS")), all_headers)

    def test_source_bytes_and_profile_constraints_are_preserved(self):
        baseline = json.loads(BASELINE.read_text())
        self.assertEqual(baseline["profile"], {"robot_model_urdf": False, "vehicle_crm": False, "vehicle_scm_gpu": False})
        root = MODEL_CMAKE.parents[3]
        for entry in baseline["files"]:
            with self.subTest(path=entry["path"]):
                payload = (root / entry["path"]).read_bytes()
                self.assertEqual(len(payload), entry["bytes"])
                self.assertEqual(hashlib.sha256(payload).hexdigest(), entry["sha256"])


if __name__ == "__main__":
    MODEL_LIST, MODEL_CMAKE, BASELINE, CATALOG, DEMO_LIST, DEMO_BUILD = [Path(value) for value in sys.argv[1:7]]
    del sys.argv[1:7]
    unittest.main()
