"""Catch omitted relative/transitive imports before a selected SDK is packaged."""

import tempfile
from pathlib import Path
import unittest

from tools.dependencies.pythonocc.imports import complete_closure, verify_selection


class GeneratedImportsTest(unittest.TestCase):
    def test_relative_public_import_and_source_helper_are_retained(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            (root / "A.py").write_text("from . import _A\nfrom .B import value\nfrom OCC.Core import C\n")
            (root / "B.py").write_text("import OCC.Wrapper.wrapper_utils\n")
            (root / "C.py").write_text("from OCC.Core.Exception import missing\nimport numpy\n")
            actual = complete_closure(root, ["A"])
            self.assertEqual(sorted(actual), ["A", "B", "C"])
            self.assertEqual(actual["A"]["modules"], ["B", "C"])
            self.assertEqual(actual["B"]["helpers"], ["OCC.Wrapper.wrapper_utils"])
            self.assertEqual(actual["C"]["helpers"], ["OCC.Core.Exception"])
            selection = {"entry_modules": ["A"], "modules": ["A", "B", "C"], "generated_imports": actual}
            verify_selection(root, selection)
            (root / "B.py").write_text("import OCC.Core.D\n")
            (root / "D.py").write_text("")
            with self.assertRaisesRegex(ValueError, "dependency closure differs"):
                verify_selection(root, selection)


if __name__ == "__main__":
    unittest.main()
