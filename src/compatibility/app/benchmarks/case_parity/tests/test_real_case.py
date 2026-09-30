"""Actual stored V5/full-native declarations must never become a GPU-win comparison."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

from benchmarks.case_parity.performance import assess

FIXTURE = Path(__file__).with_name("fixtures") / "current_unmatched"
ROOT = Path(__file__).resolve().parents[3]


class ActualMismatchTests(unittest.TestCase):
    def test_frozen_observed_model_mass_contact_and_time_disagree(self):
        request = FIXTURE / "request.json"
        pin = {"path": str(request), "sha256": hashlib.sha256(request.read_bytes()).hexdigest()}
        result = assess(pin, FIXTURE)
        self.assertEqual(result["status"], "incomparable")
        self.assertIsNone(result["gpu_speedup"])
        self.assertFalse(result["full_vehicle_requirement_met"])
        domains = {item["domain"] for item in result["mismatches"]}
        self.assertTrue({"population", "mass_inertia", "contact_history",
                         "timestep_mass_control", "initial_loads_wall",
                         "output_work_precision"}.issubset(domains))
        native = json.loads((FIXTURE / "native.json").read_text())
        current = json.loads((FIXTURE / "v5.json").read_text())
        model = json.loads((FIXTURE / "model-evidence.json").read_text())
        self.assertEqual(native["domains"]["mass_inertia"]["definition"]["total_mass_kg"],
                         model["mass"]["original_native_complete_case_kg"])
        self.assertEqual(current["domains"]["mass_inertia"]["definition"]["total_mass_kg"],
                         model["mass"]["selected_v5_kg"])

    def test_cli_writes_real_mismatch_report_and_preserves_existing_output(self):
        request = FIXTURE / "request.json"
        sha = hashlib.sha256(request.read_bytes()).hexdigest()
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "report.json"
            command = [sys.executable, "-B", "-m", "benchmarks.case_parity", str(request),
                       "--sha256", sha, "--report", str(output)]
            first = subprocess.run(command, cwd=ROOT, capture_output=True, text=True)
            self.assertEqual(first.returncode, 0, first.stderr)
            self.assertEqual(json.loads(output.read_text())["status"], "incomparable")
            before = output.read_bytes()
            second = subprocess.run(command, cwd=ROOT, capture_output=True, text=True)
            self.assertNotEqual(second.returncode, 0)
            self.assertEqual(output.read_bytes(), before)


if __name__ == "__main__":
    unittest.main()
