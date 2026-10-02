"""Behavioral inventory/plan/evidence checks; no Bazel build or physics run."""

import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

from tools.verification.chrono_inventory import digest
from tools.verification.demo_matrix.evidence import verify_build
from tools.verification.demo_matrix.plans import compiler_workers, coverage, create_plan
from tools.verification.demo_matrix.phases import prerequisite_plan
from tools.verification.demo_matrix.query import executable_sources, read_query
from tools.verification.demo_matrix.roster import build_metadata


class QueryTests(unittest.TestCase):
    def test_actual_captured_macro_alias_and_launcher_rules(self):
        records = executable_sources(read_query(FIXTURE))
        self.assertIn("src/compatibility/chrono/src/demos/core/demo_CH_coords.cpp", records["//examples/core:coordinates"]["sources"])
        self.assertIn("src/compatibility/chrono/src/demos/python/core/demo_CH_buildsystem.py", records["//examples/python/core:buildsystem"]["sources"])
        self.assertIn("src/compatibility/chrono/src/demos/csharp/core/demo_CS_CH_buildsystem.cs", records["//examples/csharp:build_system"]["sources"])
        self.assertEqual(records["//examples/fea/cuda/beams:ancf3243_sag"]["alias_for"], "@legacy_fea//lib_bin/beam_sag:test_ancf3243")

    def test_alias_cycles_reject_instead_of_inventing_exposure(self):
        rules = {"//x:a": {"kind": "alias", "attributes": {"actual": "//x:b"}, "location": ""},
                 "//x:b": {"kind": "alias", "attributes": {"actual": "//x:a"}, "location": ""}}
        with self.assertRaises(ValueError):
            executable_sources(rules)

    def test_raw_cpp_carried_as_data_does_not_claim_a_compiled_launcher(self):
        rules = {"//examples/x:launcher": {"kind": "py_binary", "location": "", "attributes": {
            "srcs": ["//tools:launch.py"], "data": ["//examples/x:uncompiled.cpp"]}}}
        self.assertNotIn("examples/x/uncompiled.cpp", executable_sources(rules)["//examples/x:launcher"]["sources"])


class PlanTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.root = Path(self.directory.name)
        subprocess.run(["git", "init", "-q", str(self.root)], check=True)
        (self.root / "BUILD.bazel").write_text("# admitted declaration\n")
        (self.root / "main.cpp").write_text("int main() { return 0; }\n")
        (self.root / "sdk").mkdir()
        self.inventory = {"programs": [{"family": "native_demo", "source": "main.cpp",
                            "current_sha256": digest((self.root / "main.cpp").read_bytes()),
                            "required_profiles": ["multicore"], "public_targets": [{"label": "//examples/x:case"}]}],
                          "captured_build_metadata": build_metadata(self.root), "input_sha256": {}}
        self.matrix = {"known_configs": ["multicore"], "cuda_arch_configs": [],
                       "sdk_sets": {"sdk": ["ROBODYNA_TEST_SDK"]},
                       "batches": [{"id": "test", "families": ["native_demo"], "targets": ["//examples/x:*"],
                                    "configs": ["multicore"], "sdk_sets": ["sdk"]}]}
        self.environment = {"schema": "robodyna.demo_operator_environment.v1", "repository": str(self.root),
                            "bazel": "/bin/true", "output_user_root": str(self.root / "bazel"),
                            "workstation_lock": str(self.root / "workstation.lock"),
                            "sdk_env": {"ROBODYNA_TEST_SDK": str(self.root / "sdk")}}
        self.inventory_path, self.matrix_path = self.root / "inventory.json", self.root / "matrix.json"
        self.inventory_path.write_text(json.dumps(self.inventory))
        self.matrix_path.write_text(json.dumps(self.matrix))

    def tearDown(self):
        self.directory.cleanup()

    def plan(self):
        return create_plan(self.inventory, self.matrix, "test", self.environment,
                           self.inventory_path, self.matrix_path, self.root / "fresh-output")

    def test_plan_expands_explicit_labels_and_never_executes(self):
        value = self.plan()
        self.assertEqual(value["targets"], ["//examples/x:case"])
        self.assertNotIn("//examples/x:*", value["command"])
        self.assertIn("--config=multicore", value["command"])
        self.assertFalse(any(arg.startswith("--disk_cache=") for arg in value["command"]))
        self.assertFalse((self.root / "fresh-output").exists())

    def test_existing_disk_cache_reaches_demo_and_native_prerequisite_commands(self):
        cache = self.root / "existing action cache"
        cache.mkdir()
        self.environment["disk_cache"] = str(cache)
        self.matrix["batches"][0]["compiler_workers"] = 2
        demo = self.plan()
        native = prerequisite_plan(demo, {"prerequisite_targets": ["//build_defs/bindings:native_core"]}, self.root / "native-output")
        option = "--disk_cache=" + str(cache)
        self.assertEqual(demo["command"].count(option), 1)
        self.assertEqual(native["command"].count(option), 1)
        self.assertIn("--jobs=2", demo["command"])
        self.assertIn("--jobs=4", native["command"])
        self.assertEqual(list(cache.iterdir()), [])

    def test_language_worker_policy_admits_measured_trial_and_fallback_only(self):
        for family in ("python_demo", "csharp_demo"):
            for workers in (1, 2):
                self.assertEqual(compiler_workers({"families": [family], "compiler_workers": workers}), workers)
            with self.assertRaisesRegex(ValueError, "one or two"):
                compiler_workers({"families": [family], "compiler_workers": 4})
        self.assertEqual(compiler_workers({"families": ["native_demo"], "compiler_workers": 4}), 4)
        for workers in (0, 5, True):
            with self.subTest(workers=workers), self.assertRaises(ValueError):
                compiler_workers({"families": ["native_demo"], "compiler_workers": workers})

    def test_relative_or_file_disk_cache_is_rejected(self):
        for value in ("relative/cache", str(self.root / "main.cpp"), None):
            self.environment["disk_cache"] = value
            with self.subTest(value=value), self.assertRaisesRegex(ValueError, "disk_cache"):
                self.plan()

    def test_missing_sdk_rejects(self):
        self.environment["sdk_env"].clear()
        with self.assertRaisesRegex(ValueError, "Missing declared SDK"):
            self.plan()

    def test_profile_change_preserves_supplied_sdk_repository_context(self):
        self.environment["sdk_env"]["ROBODYNA_OPTIX_ROOT"] = str(self.root / "sdk")
        first = self.plan()
        self.matrix["sdk_sets"]["sdk"] = ["ROBODYNA_OPTIX_ROOT"]
        second = self.plan()
        repository_options = lambda plan: [arg for arg in plan["command"] if arg.startswith("--repo_env=")]
        self.assertEqual(repository_options(first), repository_options(second))
        self.assertIn("--repo_env=ROBODYNA_OPTIX_ROOT=" + str(self.root / "sdk"), repository_options(first))
        self.assertEqual(first["required_sdk_env"], ["ROBODYNA_TEST_SDK"])
        self.assertEqual(second["required_sdk_env"], ["ROBODYNA_OPTIX_ROOT"])
        native = prerequisite_plan(first, {"prerequisite_targets": ["//build_defs/bindings:native_core"]},
                                   self.root / "native-output")
        self.assertEqual(repository_options(native), repository_options(first))
        del self.environment["sdk_env"]["ROBODYNA_TEST_SDK"]
        self.assertEqual(self.plan()["required_sdk_env"], ["ROBODYNA_OPTIX_ROOT"])

    def test_supplied_sdk_paths_are_explicit_even_when_not_required_by_batch(self):
        for value in ("relative/sdk", str(self.root / "missing-sdk"), None):
            self.environment["sdk_env"]["ROBODYNA_OPTIX_ROOT"] = value
            with self.subTest(value=value), self.assertRaisesRegex(ValueError, "Invalid supplied SDK"):
                self.plan()

    def test_missing_profile_rejects(self):
        self.matrix["batches"][0]["configs"] = []
        with self.assertRaisesRegex(ValueError, "required profile"):
            self.plan()

    def test_changed_source_and_build_metadata_require_refresh(self):
        (self.root / "main.cpp").write_text("int main() { return 1; }\n")
        with self.assertRaisesRegex(ValueError, "Program source changed"):
            self.plan()
        (self.root / "BUILD.bazel").write_text("# changed declaration\n")
        with self.assertRaisesRegex(ValueError, "fresh paired Bazel queries"):
            self.plan()

    def test_declared_but_unassigned_and_unexposed_sources_remain_visible(self):
        self.matrix["batches"][0]["targets"] = ["//other:* "]
        self.inventory["programs"].append({"family": "native_demo", "source": "pending.cpp", "public_targets": []})
        self.assertEqual(coverage(self.inventory, self.matrix), {
            "declared_but_unassigned": ["main.cpp"], "not_declared": ["pending.cpp"]})

    def test_completed_explicit_target_and_guard_are_both_required(self):
        plan = self.plan()
        guard = self.root / "guard.json"
        bep = self.root / "events.jsonl"
        guard.write_text(json.dumps({"status": "passed", "exit_code": 0, "process_scope": {"cleanup": "complete"}, "command": plan["command"],
                                    "limits": {"cpus": 8, "max_rss_gib": 16, "min_available_gib": 32, "timeout_seconds": 7200}}))
        invocation = {"schema": "robodyna.guarded_invocation.v1", "guard_sha256": digest(guard.read_bytes()),
                      "command": ["guard", "--lock", plan["resources"]["workstation_lock"], "--"] + plan["command"]}
        guard.with_name("invocation.json").write_text(json.dumps(invocation))
        bep.write_text(json.dumps({"id": {"targetCompleted": {"label": "//examples/x:case"}}, "completed": {"success": True}}) + "\n")
        self.assertEqual(verify_build(plan, guard, bep)["status"], "passed")
        bep.write_text(json.dumps({"id": {"targetSkipped": {"label": "//examples/x:case"}}}) + "\n")
        with self.assertRaisesRegex(ValueError, "Every explicitly planned target"):
            verify_build(plan, guard, bep)
        guard.write_text(json.dumps({"status": "command_failed", "exit_code": 1}))
        with self.assertRaisesRegex(ValueError, "did not close"):
            verify_build(plan, guard, bep)

    def test_over_budget_guard_is_not_accepted_as_a_qualified_build(self):
        plan = self.plan()
        guard = self.root / "guard.json"
        guard.write_text(json.dumps({"status": "passed", "exit_code": 0, "process_scope": {"cleanup": "complete"},
                                    "command": plan["command"], "limits": {"cpus": 9, "max_rss_gib": 16,
                                    "min_available_gib": 32, "timeout_seconds": 7200}}))
        with self.assertRaisesRegex(ValueError, "resource limits"):
            verify_build(plan, guard, self.root / "unused-events")


if __name__ == "__main__":
    FIXTURE = Path(sys.argv.pop(1)) if len(sys.argv) > 1 and not sys.argv[1].startswith("-") else Path(__file__).with_name("query_fixture.xml")
    unittest.main()
