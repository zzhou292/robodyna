"""Verify resume journals and source drift without launching a build."""

import json
from pathlib import Path
import subprocess
import tempfile
import unittest

from tools.dependencies.cuda_math import file_hash
from tools.verification.demo_matrix.evidence import verify_build
from tools.verification.demo_matrix.executor import accepted_attempt, next_attempt, save
from tools.verification.demo_matrix.snapshots import require_unchanged, source_snapshot


class ExecutorTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.repo = self.root / "repository"
        self.repo.mkdir()
        subprocess.run(["git", "init", "-q", str(self.repo)], check=True)
        (self.repo / "kernel.cpp").write_text("int value() { return 1; }\n")
        (self.repo / "BUILD.bazel").write_text("# initial source owner\n")

    def tearDown(self):
        self.temp.cleanup()

    def test_resume_detects_implementation_changes_not_only_main_changes(self):
        original = source_snapshot(self.repo)
        require_unchanged(self.repo, original)
        (self.repo / "kernel.cpp").write_text("int value() { return 2; }\n")
        with self.assertRaisesRegex(ValueError, "Source/build inputs changed"):
            require_unchanged(self.repo, original)

    def test_failed_attempts_are_preserved_when_new_attempt_is_created(self):
        parent = self.root / "attempts"
        first = next_attempt(parent)
        (first / "failure.json").write_text('{"status":"failed"}\n')
        second = next_attempt(parent)
        self.assertNotEqual(first, second)
        self.assertEqual((first / "failure.json").read_text(), '{"status":"failed"}\n')

    def test_dependency_lock_foreign_build_and_shader_changes_are_inputs(self):
        for name in ("MODULE.bazel.lock", "CMakeLists.txt", "build.sh", "kernel.S", "sensor.rgen", "render.frag"):
            path = self.repo / name
            path.write_text("initial input\n")
            original = source_snapshot(self.repo)
            path.write_text("modified input\n")
            with self.subTest(name=name), self.assertRaisesRegex(ValueError, "Source/build inputs changed"):
                require_unchanged(self.repo, original)

    def test_completed_resume_requires_same_inventory_batch_and_receipts(self):
        parent = self.root / "batches" / "one"
        output = next_attempt(parent) / "outputs"
        output.mkdir()
        inventory = {"programs": [{"family": "native_demo", "source": "main.cpp",
                                   "public_targets": [{"label": "//examples/x:case"}]}]}
        batch = {"id": "one", "families": ["native_demo"], "targets": ["//examples/x:*"]}
        inventory_path, matrix_path = self.root / "inventory.json", self.root / "matrix.json"
        save(inventory_path, inventory)
        save(matrix_path, {"batches": [batch]})
        plan = {"batch": "one", "targets": ["//examples/x:case"], "configs": [], "command": ["declared-bazel", "build", "//examples/x:case"],
                "inventory_sha256": file_hash(inventory_path), "matrix_sha256": file_hash(matrix_path),
                "resources": {"affinity_cpus": 8, "rss_gib": 16, "minimum_available_ram_gib": 32,
                              "timeout_seconds": 7200, "workstation_lock": "/declared/workstation.lock"}}
        save(output / "plan.json", plan)
        save(output / "guard.json", {"status": "passed", "exit_code": 0,
                                     "process_scope": {"cleanup": "complete"}, "command": plan["command"],
                                     "limits": {"cpus": 8, "max_rss_gib": 16, "min_available_gib": 32, "timeout_seconds": 7200}})
        save(output / "invocation.json", {"schema": "robodyna.guarded_invocation.v1", "guard_sha256": file_hash(output / "guard.json"),
                                          "command": ["guard", "--lock", "/declared/workstation.lock", "--"] + plan["command"]})
        (output / "build-events.jsonl").write_text(json.dumps({
            "id": {"targetCompleted": {"label": "//examples/x:case"}}, "completed": {"success": True}}) + "\n")
        save(output / "qualification.json", verify_build(plan, output / "guard.json", output / "build-events.jsonl"))
        self.assertEqual(accepted_attempt(parent, batch, inventory, inventory_path, matrix_path), output)
        inventory_path.write_text('{"modified":true}\n')
        with self.assertRaisesRegex(ValueError, "current admitted batch"):
            accepted_attempt(parent, batch, inventory, inventory_path, matrix_path)


if __name__ == "__main__":
    unittest.main()
