"""Manual metadata regression against the preserved, real 100 ms delivery."""

import os
from pathlib import Path
import unittest

from viewer.file_integrity import sha256_file
from src.simulation.driver.inspection import inspect_run


class DeliveredArchiveMetadata(unittest.TestCase):
    def test_real_completed_archive_is_inspected_without_mutation_or_new_replay_claim(self):
        root = Path(os.environ["ROBODYNA_ACCEPTED_100MS"]).absolute()
        guard = Path(os.environ["ROBODYNA_ACCEPTED_100MS_GUARD"]).absolute()
        files = [guard, root / "summary.json", root / "viewer-input.json", root / "archive/manifest.json",
                 root / "archive/configuration.json", root / "archive/frame-index.json"]
        before = {str(path): sha256_file(path) for path in files}
        result = inspect_run(root, guard)
        self.assertTrue(result["horizon_complete"])
        self.assertEqual(result["accepted_intervals"], 666667)
        self.assertEqual(result["actual_time_s"], 0.10000005000066203)
        self.assertEqual(result["saved_states"], 301)
        self.assertFalse(result["physical_restart"])
        self.assertFalse(result["full_cpp_replay_verified"])
        self.assertEqual(before, {str(path): sha256_file(path) for path in files})


if __name__ == "__main__":
    unittest.main()
