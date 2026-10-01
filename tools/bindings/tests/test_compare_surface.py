"""Comparison admission: public behavior changes and missing output must fail."""

from pathlib import Path
import tempfile
import unittest

from tools.bindings.compare_surface import compare


class SurfaceComparison(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        root = Path(self.temp.name)
        self.old, self.new = root / "old", root / "new"
        self.proxy = 'class Body:\n    """Generated documentation."""\n    def SetMass(self, mass):\n        return native_set(self, mass)\nVALUE = 3\n'
        for directory in (self.old, self.new):
            (directory / "python").mkdir(parents=True)
            (directory / "csharp").mkdir()
            (directory / "python/core.py").write_text(self.proxy)
            (directory / "python/stderr.log").write_text("")
            (directory / "csharp/Body.cs").write_text("public class Body {}\n")
            (directory / "csharp/stderr.log").write_text("")

    def test_equal_output_and_documentation_only_changes(self):
        self.assertTrue(compare(self.old, self.new)["passed"])
        (self.new / "python/core.py").write_text(self.proxy.replace("Generated documentation.", "New C++ spelling."))
        self.assertTrue(compare(self.old, self.new)["passed"])

    def test_changed_signature_or_method_body_is_not_documentation(self):
        for changed in (self.proxy.replace("self, mass):", "self):"),
                        self.proxy.replace("native_set(self, mass)", "native_set(self, mass * 2)")):
            (self.new / "python/core.py").write_text(changed)
            result = compare(self.old, self.new)
            self.assertFalse(result["passed"])
            self.assertIn("Body", result["python"]["changed"])

    def test_removed_and_added_public_names_are_reported(self):
        (self.new / "python/core.py").write_text(self.proxy.replace("VALUE = 3", "REPLACEMENT = 3"))
        result = compare(self.old, self.new)
        self.assertEqual(result["python"]["removed"], ["VALUE"])
        self.assertEqual(result["python"]["added"], ["REPLACEMENT"])
        self.assertFalse(result["passed"])

    def test_csharp_or_diagnostic_changes_fail(self):
        (self.new / "csharp/Body.cs").write_text("public class Body { public int Added; }\n")
        result = compare(self.old, self.new)
        self.assertFalse(result["passed"])
        self.assertEqual(result["csharp"]["changed"], ["Body.cs"])
        (self.new / "csharp/Body.cs").write_text("public class Body {}\n")
        (self.new / "python/stderr.log").write_text("Warning: unknown base class\n")
        self.assertFalse(compare(self.old, self.new)["passed"])

    def test_missing_selected_outputs_cannot_pass_as_an_empty_comparison(self):
        (self.new / "csharp/Body.cs").unlink()
        with self.assertRaisesRegex(ValueError, "no generated proxy"):
            compare(self.old, self.new)

    def test_declared_snapshot_roots_do_not_hide_warning_changes(self):
        old_root, new_root = self.old / "inputs", self.new / "inputs"
        for directory, root in ((self.old, old_root), (self.new, new_root)):
            (directory / "python/stderr.log").write_text(str(root) + "/Class.h:50: Warning 402: incomplete base\n")
        self.assertFalse(compare(self.old, self.new)["passed"])
        result = compare(self.old, self.new, baseline_inputs=old_root, candidate_inputs=new_root)
        self.assertTrue(result["passed"])
        self.assertNotEqual(result["diagnostics"]["baseline"], result["diagnostics"]["candidate"])
        for changed in ("Class.h:51: Warning 402: incomplete base", "Other.h:50: Warning 402: incomplete base",
                        "Class.h:50: Warning 401: missing member"):
            (self.new / "python/stderr.log").write_text(str(new_root) + "/" + changed + "\n")
            self.assertFalse(compare(self.old, self.new, baseline_inputs=old_root, candidate_inputs=new_root)["passed"])

    def test_fea_selects_its_real_python_module_without_inventing_csharp(self):
        for directory in (self.old, self.new):
            (directory / "python/fea.py").write_text(self.proxy)
        self.assertTrue(compare(self.old, self.new, "fea", ("python",))["passed"])
        with self.assertRaisesRegex(ValueError, "separate C# FEA"):
            compare(self.old, self.new, "fea", ("python", "csharp"))


if __name__ == "__main__":
    unittest.main()
