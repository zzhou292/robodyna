"""Failure-oriented tests for migration admission, independent of CUDA/Bazel."""

import hashlib
from pathlib import Path
import tempfile
import unittest

from build_defs.source_boundary import declarations, inspect_workspace, workspace_path
from build_defs.build_overlays import verify_build_overlay


class SourceBoundaryTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        files = {
            "MODULE.bazel": 'module(name="robodyna")\n'
                'local_repository(name="legacy_fea",path="src/fea/legacy")\n'
                + "".join(f'single_version_override(module_name="{name}",patch_strip=1)\n'
                          for name in ("rules_cuda", "rules_foreign_cc", "googletest")),
            ".bazelversion": "9.2.0\n",
            ".bazelrc": "build --jobs=4\n",
            ".bazelignore": "src/fea/legacy\n",
            "src/fea/BUILD.bazel": 'alias(name="owner",actual="@legacy_fea//model:owner")\n',
            "src/fea/legacy/model/BUILD.bazel": 'cc_library(name="owner")\n',
            "src/fea/legacy/model/state.h": "// Qualified physical state.\n",
        }
        for name, value in files.items():
            path = self.root / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(value)
        self.contract = {"source_root": "src/fea/legacy", "repository_name": "legacy_fea",
                         "dependency_versions": {}, "patches": {}, "source_files": {
                             "model/state.h": hashlib.sha256(files["src/fea/legacy/model/state.h"].encode()).hexdigest()}}

    def test_valid_relocated_checkout(self):
        self.assertEqual(inspect_workspace(self.root, self.contract), [])

    def test_reject_source_change(self):
        (self.root / "src/fea/legacy/model/state.h").write_text("changed arithmetic")
        self.assertTrue(any("Qualified source changed" in error
                            for error in inspect_workspace(self.root, self.contract)))

    def test_reject_missing_alias_target(self):
        (self.root / "src/fea/legacy/model/BUILD.bazel").write_text('cc_library(name="other")\n')
        self.assertTrue(any("Missing inherited target" in error
                            for error in inspect_workspace(self.root, self.contract)))

    def test_reject_absolute_and_parent_paths(self):
        for value in ("/tmp/foreign-source", "../foreign-source"):
            with self.subTest(value=value), self.assertRaises(ValueError):
                workspace_path(self.root, value)

    def test_reject_symlink_outside_checkout(self):
        with tempfile.TemporaryDirectory() as other:
            (self.root / "outside").symlink_to(other, target_is_directory=True)
            with self.assertRaises(ValueError):
                workspace_path(self.root, "outside/source")

    def test_reject_global_fast_math(self):
        (self.root / ".bazelrc").write_text("build --copt=-ffast-math\n")
        self.assertTrue(any("Numerical policies" in error
                            for error in inspect_workspace(self.root, self.contract)))

    def test_does_not_evaluate_computed_attributes(self):
        path = self.root / "declaration.bazel"
        path.write_text('cc_library(name="owner",srcs=unknown_function())\n')
        self.assertEqual(declarations(path), [("cc_library", {"name": "owner"})])

    def test_reviewed_build_overlay_reconstructs_baseline(self):
        before = b'cc_library(name="owner", hdrs=["State.h"])\n'
        after = b'cc_library(name="owner", hdrs=["State.h", "Internal.h"])\n'
        entry = {"baseline_sha256": hashlib.sha256(before).hexdigest(),
                 "overlay_sha256": hashlib.sha256(after).hexdigest(),
                 "replacements": [{"before": 'hdrs=["State.h"]',
                                   "after": 'hdrs=["State.h", "Internal.h"]'}]}
        self.assertEqual(verify_build_overlay("module/BUILD.bazel", after, entry), entry["baseline_sha256"])
        with self.assertRaisesRegex(ValueError, "Only BUILD"):
            verify_build_overlay("module/Physics.cu", after, entry)
        unreviewed = after + b'cc_library(name="unreviewed")\n'
        entry["overlay_sha256"] = hashlib.sha256(unreviewed).hexdigest()
        with self.assertRaisesRegex(ValueError, "does not reconstruct"):
            verify_build_overlay("module/BUILD.bazel", unreviewed, entry)


if __name__ == "__main__":
    unittest.main()
