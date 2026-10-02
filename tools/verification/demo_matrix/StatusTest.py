"""Validate read-only progress from closed receipts, without executing Bazel."""

import json
from pathlib import Path
import tempfile
import unittest

from tools.dependencies.cuda_math import file_hash
from tools.verification.demo_matrix.evidence import verify_build
from tools.verification.demo_matrix.executor import save
from tools.verification.demo_matrix.status import build_status


class StatusTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)

    def tearDown(self):
        self.temp.cleanup()

    def closed(self, attempt=1, phase="demo_targets", subdirectory="outputs"):
        output = self.root / "batches/one" / ("attempt-%04d" % attempt) / subdirectory
        output.mkdir(parents=True)
        plan = {"batch": "one", "phase": phase, "targets": ["//examples/x:case"], "configs": [],
                "command": ["declared-bazel", "build", "//examples/x:case"],
                "inventory_sha256": "inventory-pin", "matrix_sha256": "matrix-pin",
                "resources": {"affinity_cpus": 8, "rss_gib": 16, "minimum_available_ram_gib": 32,
                              "timeout_seconds": 7200, "workstation_lock": "/declared/workstation.lock"}}
        save(output / "plan.json", plan)
        save(output / "guard.json", {"status": "passed", "exit_code": 0,
            "process_scope": {"cleanup": "complete"}, "command": plan["command"],
            "limits": {"cpus": 8, "max_rss_gib": 16, "min_available_gib": 32, "timeout_seconds": 7200}})
        save(output / "invocation.json", {"schema": "robodyna.guarded_invocation.v1",
            "guard_sha256": file_hash(output / "guard.json"),
            "command": ["guard", "--lock", "/declared/workstation.lock", "--"] + plan["command"]})
        (output / "build-events.jsonl").write_text(json.dumps({
            "id": {"targetCompleted": {"label": "//examples/x:case"}}, "completed": {"success": True}}) + "\n")
        save(output / "qualification.json", verify_build(plan, output / "guard.json", output / "build-events.jsonl"))
        return output

    def test_closed_batch_is_visible_before_final_controller_summary(self):
        output = self.closed()
        self.assertFalse((self.root / "runs").exists())
        status = build_status(self.root)
        self.assertEqual(status["batches"]["one"]["evidence"], str(output))
        self.assertEqual(status["compiled_labels"], ["//examples/x:case"])

    def test_prerequisites_and_unfinished_outputs_are_not_demo_success(self):
        self.closed(phase="native_prerequisites", subdirectory="native-prerequisites")
        output = self.root / "batches/two/attempt-0001/outputs"
        output.mkdir(parents=True)
        (output / "plan.json").write_text('{"status":"not-yet-closed"}\n')
        self.assertEqual(build_status(self.root)["batches"], {})

    def test_later_failed_attempt_preserves_the_last_success(self):
        first = self.closed()
        failed = self.root / "batches/one/attempt-0002/outputs"
        failed.mkdir(parents=True)
        save(failed / "guard.json", {"status": "command_failed", "exit_code": 1})
        self.assertEqual(build_status(self.root)["batches"]["one"]["evidence"], str(first))
        last = self.closed(attempt=3)
        self.assertEqual(build_status(self.root)["batches"]["one"]["evidence"], str(last))

    def test_tampered_closed_receipt_fails_instead_of_becoming_invisible(self):
        output = self.closed()
        guard = json.loads((output / "guard.json").read_text())
        guard["limits"]["cpus"] = 9
        (output / "guard.json").write_text(json.dumps(guard))
        with self.assertRaisesRegex(ValueError, "resource limits"):
            build_status(self.root)

    def test_malformed_qualification_fails_closed(self):
        output = self.closed()
        (output / "qualification.json").write_text('{"status":')
        with self.assertRaises(ValueError):
            build_status(self.root)

    def test_foreign_batch_or_prerequisite_claim_cannot_count_as_demo_pass(self):
        self.closed(phase="native_prerequisites")
        with self.assertRaisesRegex(ValueError, "identify this demo batch"):
            build_status(self.root)


if __name__ == "__main__":
    unittest.main()
