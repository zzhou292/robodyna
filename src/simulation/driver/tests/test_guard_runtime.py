"""Execute the real guard's help path with Python safe-path; no monitored job."""

import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
import json

from viewer.file_integrity import sha256_file
from src.simulation.driver.runtime import clean_environment, monitored_command, runtime_file, watchdog_environment
from src.simulation.driver.watchdog import select_watchdog_interpreter

HELPERS = ("run_bounded.py", "bounded_history.py", "bounded_gpu.py", "bounded_gpu_query.py",
           "bounded_session.py", "bounded_stop.py")


def selected_guard():
    try:
        from python.runfiles import runfiles  # noqa: F401
    except ImportError:
        source = Path(__file__).resolve().parents[4] / "src/fea/legacy/tools/run_bounded.py"
        return runtime_file("legacy_fea/tools/run_bounded.py", source)
    return runtime_file("legacy_fea/tools/run_bounded.py")


class GuardRuntimeTests(unittest.TestCase):
    def execute_help(self, guard):
        parent = guard.resolve().parent
        before = {name: sha256_file(parent / name) for name in HELPERS}
        environment = clean_environment(0)
        environment["PYTHONSAFEPATH"] = "1"
        environment["PYTHONPATH"] = ""
        environment = watchdog_environment(environment, guard)
        # The override is only for standalone reproduction with the exact
        # packaged interpreter; ordinary Bazel tests already use that runtime.
        interpreter = os.environ.get("ROBODYNA_GUARD_TEST_INTERPRETER", sys.executable)
        result = subprocess.run([interpreter, "-B", str(guard), "--help"], env=environment,
                                capture_output=True, text=True, timeout=10, check=False)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("--max-gpu-growth-gib", result.stdout)
        self.assertIn("--report", result.stdout)
        self.assertNotIn("Traceback", result.stderr)
        self.assertEqual(before, {name: sha256_file(parent / name) for name in HELPERS})

    def test_real_selected_guard_loads_all_helpers_in_safe_path_context(self):
        self.execute_help(selected_guard())

    def test_explicit_symlink_guard_override_uses_its_actual_siblings(self):
        with tempfile.TemporaryDirectory() as temporary:
            alias = Path(temporary) / "guard-override.py"
            alias.symlink_to(selected_guard())
            self.execute_help(alias)

    def test_real_monitor_runs_true_and_seals_complete_cleanup(self):
        guard = selected_guard()
        environment = clean_environment(0)
        environment["PYTHONSAFEPATH"] = "1"
        environment["PYTHONHOME"] = "/deliberately-incompatible-cli-runtime"
        selected = select_watchdog_interpreter(guard, environment,
            current_interpreter=os.environ.get("ROBODYNA_GUARD_TEST_INTERPRETER", sys.executable))
        self.assertEqual(selected["capability"], "owned_session_v1")
        child_environment = watchdog_environment(environment, guard)
        self.assertNotIn("PYTHONHOME", child_environment)
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            limits = dict(cpu_threads=1, rss_bytes=256 << 20, minimum_available_ram_bytes=32 << 30,
                          timeout_s=10, workstation_lock=str(root / "tiny-unit.lock"))
            command = monitored_command(guard, limits, root / "guard.json", ["/bin/true"], selected["path"])
            completed = subprocess.run(command, env=child_environment, capture_output=True, text=True,
                                       timeout=15, check=False)
            self.assertEqual(completed.returncode, 0, completed.stderr + completed.stdout)
            report = json.loads((root / "guard.json").read_text())
            self.assertEqual(report["status"], "passed")
            self.assertEqual(report["exit_code"], 0)
            self.assertEqual(report["process_scope"]["cleanup"], "complete")
            self.assertEqual(report["command"], ["/bin/true"])

    def test_unsupported_explicit_interpreter_is_not_silently_replaced(self):
        with self.assertRaisesRegex(ValueError, "No watchdog Python"):
            select_watchdog_interpreter(selected_guard(), clean_environment(0), explicit="/bin/false")


if __name__ == "__main__":
    unittest.main()
