import importlib
import os
from pathlib import Path
import tempfile
import unittest

from tools.managed.environment import mono_environment

collect_sources = importlib.import_module("tools.managed.compile").collect_sources


class ManagedAdmissionTest(unittest.TestCase):
    def test_selected_sdk_replaces_inherited_mono_and_native_search_paths(self):
        env = mono_environment({"MONO_PATH": "/old", "MONO_ENV_OPTIONS": "--debug", "LD_PRELOAD": "/old.so",
                                "PYTHONPATH": "/other", "RUNFILES_DIR": "/caller", "PATH": "/bin"},
                               "/sdk", ["/product"], ["/native"])
        self.assertEqual(env["MONO_PATH"], "/sdk/usr/lib/mono/4.5" + os.pathsep + "/product")
        self.assertEqual(env["LD_LIBRARY_PATH"], "/sdk/usr/lib" + os.pathsep + "/native")
        for name in ("MONO_ENV_OPTIONS", "LD_PRELOAD", "PYTHONPATH", "RUNFILES_DIR"):
            self.assertNotIn(name, env)
        self.assertEqual(env["PATH"], "/bin")

    def test_sources_are_ordered_and_ambiguous_cross_module_names_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "core").mkdir()
            (root / "core/B.cs").write_text("class B {}")
            (root / "core/A.cs").write_text("class A {}")
            self.assertEqual([path.name for path in collect_sources([root / "core"])], ["A.cs", "B.cs"])
            (root / "A.cs").write_text("class A {}")
            with self.assertRaisesRegex(ValueError, "Duplicate"):
                collect_sources([root / "core", root / "A.cs"])

    def test_empty_source_tree_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaisesRegex(ValueError, "No declared"):
                collect_sources([directory])


if __name__ == "__main__":
    unittest.main()
