import hashlib
from pathlib import Path
import tempfile
import unittest

from tools.managed.compose import compose


class CompositionTest(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.root = Path(self.directory.name)
        for name in ("core", "vsg"):
            (self.root / name).mkdir()

    def modules(self):
        return [(name, self.root / name) for name in ("core", "vsg")]

    def test_equal_shared_proxy_preserves_first_owner_and_unique_types(self):
        for name in ("core", "vsg"):
            (self.root / name / "Shared.cs").write_text("class Shared {}")
        (self.root / "vsg/Visual.cs").write_text("class Visual {}")
        result = compose(self.modules(), self.root / "out")
        self.assertEqual(sorted(path.name for path in (self.root / "out").iterdir()), ["Shared.cs", "Visual.cs"])
        self.assertEqual(result["selected"]["Shared.cs"]["module"], "core")
        self.assertTrue(result["duplicates"][0]["identical"])

    def test_differing_proxy_needs_exact_reviewed_hash_pair(self):
        left = "class Shared { public int Old; }"
        right = "class Shared { public int New; }"
        (self.root / "core/Shared.cs").write_text(left)
        (self.root / "vsg/Shared.cs").write_text(right)
        with self.assertRaisesRegex(ValueError, "requires explicit review"):
            compose(self.modules(), self.root / "rejected")
        approval = {"file": "Shared.cs", "kept_module": "core", "discarded_module": "vsg",
                    "kept_sha256": hashlib.sha256(left.encode()).hexdigest(),
                    "discarded_sha256": hashlib.sha256(right.encode()).hexdigest()}
        result = compose(self.modules(), self.root / "out", [approval])
        self.assertFalse(result["duplicates"][0]["identical"])
        self.assertEqual((self.root / "out/Shared.cs").read_text(), left)
        (self.root / "vsg/Shared.cs").write_text(right + " ")
        with self.assertRaisesRegex(ValueError, "requires explicit review"):
            compose(self.modules(), self.root / "stale", [approval])

    def test_repeated_module_is_not_silently_coalesced(self):
        (self.root / "core/Shared.cs").write_text("class Shared {}")
        with self.assertRaisesRegex(ValueError, "repeated"):
            compose([self.modules()[0], self.modules()[0]], self.root / "out")

    def test_bazel_empty_tree_is_admitted_but_prior_output_is_preserved(self):
        (self.root / "core/Shared.cs").write_text("class Shared {}")
        output = self.root / "precreated"
        output.mkdir()
        compose([self.modules()[0]], output)
        with self.assertRaisesRegex(ValueError, "empty directory"):
            compose([self.modules()[0]], output)
        self.assertEqual((output / "Shared.cs").read_text(), "class Shared {}")

    def test_output_directory_symlink_is_rejected(self):
        (self.root / "core/Shared.cs").write_text("class Shared {}")
        target = self.root / "elsewhere"
        target.mkdir()
        output = self.root / "link"
        output.symlink_to(target, target_is_directory=True)
        with self.assertRaisesRegex(ValueError, "symlink"):
            compose([self.modules()[0]], output)
        self.assertEqual(list(target.iterdir()), [])


if __name__ == "__main__":
    unittest.main()
