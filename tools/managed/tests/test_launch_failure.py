"""Receipt behavior when a declared executable disappears before child creation."""

import json
from pathlib import Path
import tempfile
import unittest

from tools.managed.launch import run_demo


class LaunchFailureTest(unittest.TestCase):
    def test_process_creation_failure_keeps_request_and_seals_failure(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            inputs = {name: root / name for name in
                      ("assembly", "managed", "native_wrapper", "native_core", "data_anchor")}
            # Deliberately absent executable: this calls the real subprocess API
            # and never invokes a mock managed or physics implementation.
            inputs["mono"] = root / "removed-sdk/usr/bin/mono-sgen"
            output = root / "run"
            with self.assertRaises(FileNotFoundError):
                run_demo(inputs, output)
            request = json.loads((output / "request.json").read_text())
            result = json.loads((output / "result.json").read_text())
            self.assertEqual(request["status"], "running")
            self.assertEqual(result["status"], "failed")
            self.assertIsNone(result["exit_code"])
            self.assertEqual(result["native_backend_paths"], [])
            self.assertIn("mono-sgen", result["error"])
            self.assertTrue((output / "stdout.log").is_file())
            with self.assertRaises(FileExistsError):
                run_demo(inputs, output)
            self.assertEqual(json.loads((output / "result.json").read_text()), result)


if __name__ == "__main__":
    unittest.main()
