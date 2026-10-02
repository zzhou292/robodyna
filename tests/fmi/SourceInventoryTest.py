"""Keep FMI executables, exported models and retained sources distinct."""

import ast
import hashlib
import json
from pathlib import Path
import shlex
import sys
import unittest

from tools.migration.source_transform import index_entries, original_bytes
from tools.verification.cmake_evidence import commands


class SourceInventoryTest(unittest.TestCase):
    def test_eight_original_model_definitions_match_their_cmake_identifiers(self):
        manifest = json.loads(EXPORTS.read_text())
        self.assertEqual(manifest["counts"], {"fmus": 8, "drivers": 5})
        self.assertEqual(len(manifest["exports"]), 8)
        self.assertEqual(len({row["identifier"] for row in manifest["exports"]}), 8)
        for row in manifest["exports"]:
            values = {}
            for command in commands((SOURCE_ROOT / row["cmake"]).read_text()):
                if command["command"] == "set":
                    words = shlex.split(command["arguments"])
                    if len(words) >= 2:
                        values[words[0]] = words[1]
            expected = values.get("COMPONENT_NAME", values.get("FMU_MODEL_IDENTIFIER"))
            self.assertEqual(row["identifier"], expected)
            self.assertTrue((SOURCE_ROOT / row["source"]).is_file())
            self.assertTrue((SOURCE_ROOT / row["header"]).is_file())

    def test_real_driver_entrypoints_are_separate_from_shared_models(self):
        models, drivers = [], []
        for node in ast.parse(BUILD.read_text()).body:
            if not isinstance(node, ast.Expr) or not isinstance(node.value, ast.Call):
                continue
            call = node.value
            kind = getattr(call.func, "id", "")
            if kind not in ("native_fmu", "native_fmi_driver"):
                continue
            values = {kw.arg: ast.literal_eval(kw.value) for kw in call.keywords}
            (models if kind == "native_fmu" else drivers).append(values)
        self.assertEqual(len(models), 8)
        self.assertEqual(len(drivers), 5)
        expected = {"//src/compatibility/chrono:" + path.relative_to(SOURCE_ROOT).as_posix()
                    for path in (SOURCE_ROOT / "src/demos/fmi").rglob("demo_*.cpp")}
        expected.add("//src/compatibility/chrono:template_project_fmi2/demo_FmuComponentChrono.cpp")
        self.assertEqual({row["source"] for row in drivers}, expected)
        self.assertEqual({row["identifier"] for row in models},
                         {row["identifier"] for row in json.loads(EXPORTS.read_text())["exports"]})

    def test_source_snapshot_and_exact_metadata_inverses(self):
        baseline = json.loads(BASELINE.read_text())
        self.assertEqual(baseline["schema"], "robodyna.retained_fmi_source_snapshot.v1")
        recipes = index_entries(json.loads(RECIPES.read_text()))
        self.assertEqual(len(recipes), 3)
        workspace = SOURCE_ROOT.parents[2]
        for row in baseline["files"]:
            path = "src/compatibility/chrono/" + row["path"]
            before = original_bytes(workspace, recipes[path]) if path in recipes else (SOURCE_ROOT / row["path"]).read_bytes()
            with self.subTest(path=row["path"]):
                self.assertEqual(len(before), row["bytes"])
                self.assertEqual(hashlib.sha256(before).hexdigest(), row["sha256"])

    def test_donor_is_the_recorded_gitlink_and_authenticated_archive(self):
        donor = json.loads(DEPENDENCY.read_text())
        self.assertEqual(donor["recorded_gitlink"], "cbf2a58ba9518e6230e831ff0b3640f9a0b4905a")
        self.assertEqual(donor["sha256"], "56eec5cf264b357d43e2087396106517ccfd0d8a6b187ac770ecba272f94efe0")
        self.assertEqual(donor["bytes"], 1927164)


if __name__ == "__main__":
    EXPORTS, BUILD, BASELINE, RECIPES, DEPENDENCY, CMAKE = [Path(value) for value in sys.argv[1:7]]
    SOURCE_ROOT = CMAKE.parents[2]
    del sys.argv[1:7]
    unittest.main()
