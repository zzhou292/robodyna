from pathlib import Path
import tempfile
import unittest
import sys
from unittest.mock import patch

from src.simulation.driver.job import launch
from src.simulation.driver.jsonio import read_json
from src.simulation.driver.runtime import clean_environment
from .fixture import case_fixture


class LaunchTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        case_fixture(self.root)
        self.backend = self.root / "native"
        self.backend.write_bytes(b"not executed by this host test")
        self.backend.chmod(0o700)
        self.guard = self.root / "guard.py"
        self.guard.write_text("# fixture; subprocess is mocked\n")
        selector = patch("src.simulation.driver.job.select_watchdog_interpreter",
                         return_value=dict(path=sys.executable, capability="mock-only-no-process"))
        selector.start()
        self.addCleanup(selector.stop)

    def call(self, destination):
        return launch(self.root / "case.json", self.root / "resources.json", destination,
                      "plan", self.backend, self.guard)

    def test_missing_backend_creates_no_output(self):
        self.backend.unlink()
        with self.assertRaises(ValueError):
            self.call(self.root / "output")
        self.assertFalse((self.root / "output").exists())

    def test_existing_output_is_never_reused(self):
        output = self.root / "output"
        output.mkdir()
        sentinel = output / "preserve"
        sentinel.write_text("unchanged")
        with self.assertRaisesRegex(ValueError, "new directory"):
            self.call(output)
        self.assertEqual(sentinel.read_text(), "unchanged")

    def test_output_cannot_be_created_inside_canonical_source(self):
        with self.assertRaisesRegex(ValueError, "immutable canonical"):
            self.call(self.root / "canonical/output")
        self.assertFalse((self.root / "canonical/output").exists())

    def test_plan_launch_records_real_request_and_keeps_guard_failure(self):
        with patch("src.simulation.driver.job.subprocess.run") as run:
            run.return_value.returncode = 125
            result, code = self.call(self.root / "output")
        self.assertEqual(code, 125)
        self.assertEqual(result["return_code"], 125)
        argv = run.call_args.args[0]
        self.assertIn("--request-sha256", argv)
        self.assertNotIn("--gtest_filter", " ".join(argv))
        request = read_json(self.root / "output/request.json")
        self.assertEqual(request["resources"]["solid_worker_blocks"], 32)
        self.assertEqual(request["run"]["fixed_dt_s"], 1.5e-7)
        self.assertFalse((self.root / "output/accepted").exists())
        self.assertFalse(read_json(self.root / "output/launch.json")["physics_prepared"])

    def test_qualification_environment_cannot_override_manifest(self):
        with patch.dict("os.environ", {"ROBO_NATIVE_FIXED_DT_S": "1", "GTEST_REPEAT": "100", "CUDA_LAUNCH_BLOCKING": "1"}):
            result = clean_environment(2)
        self.assertNotIn("ROBO_NATIVE_FIXED_DT_S", result)
        self.assertNotIn("GTEST_REPEAT", result)
        self.assertNotIn("CUDA_LAUNCH_BLOCKING", result)
        self.assertEqual(result["CUDA_VISIBLE_DEVICES"], "2")

    def test_zero_exit_without_native_and_guard_receipts_is_not_success(self):
        with patch("src.simulation.driver.job.subprocess.run") as run:
            run.return_value.returncode = 0
            result, code = self.call(self.root / "output")
        self.assertEqual(code, 1)
        self.assertFalse(result["product_result_verified"])
        self.assertIn("verification_error", result)


if __name__ == "__main__":
    unittest.main()
