import hashlib
import tempfile
import unittest
from pathlib import Path

from tools.migration.source_transform import index_entries, original_bytes, replace_code_token


class SourceTransformationTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.original = b"double Old::Energy(double x) { return x * x; }\n"
        (self.root / "new.cpp").write_bytes(self.original.replace(b"Old", b"New"))
        self.entry = {"original_path": "old.cpp", "canonical_path": "new.cpp",
                      "original_sha256": hashlib.sha256(self.original).hexdigest(),
                      "replacements": [{"before": "Old", "after": "New", "count": 1, "word": True}]}

    def test_exact_rename_restores_immutable_original(self):
        self.assertEqual(original_bytes(self.root, self.entry), self.original)

    def test_code_token_rewrite_preserves_includes_comments_and_strings(self):
        source = '#include "Old.h"\n#include <Old.h>\n// Old\n/* Old */\n"Old"; Old value;\n'
        transformed, count = replace_code_token(source, "Old", "other::Old")
        self.assertEqual(count, 1)
        self.assertEqual(transformed, source.replace("Old value", "other::Old value"))
        self.assertEqual(replace_code_token(transformed, "other::Old", "Old"), (source, 1))

    def test_arithmetic_change_cannot_be_accepted_by_rename(self):
        (self.root / "new.cpp").write_text("double New::Energy(double x) { return x + x; }\n")
        with self.assertRaisesRegex(ValueError, "beyond"):
            original_bytes(self.root, self.entry)

    def test_old_implementation_must_not_remain(self):
        (self.root / "old.cpp").write_bytes(self.original)
        with self.assertRaisesRegex(ValueError, "still exists"):
            original_bytes(self.root, self.entry)

    def test_changed_forwarder_and_missing_rename_reject(self):
        old = self.root / "old.cpp"
        old.write_text("forwarder\n")
        self.entry["forwarding_sha256"] = hashlib.sha256(old.read_bytes()).hexdigest()
        self.assertEqual(original_bytes(self.root, self.entry), self.original)
        old.write_text("different forwarder\n")
        with self.assertRaisesRegex(ValueError, "forwarding"):
            original_bytes(self.root, self.entry)
        old.write_text("forwarder\n")
        (self.root / "new.cpp").write_bytes(self.original)
        with self.assertRaisesRegex(ValueError, "count"):
            original_bytes(self.root, self.entry)

    def test_duplicate_entries_and_path_escape_reject(self):
        with self.assertRaisesRegex(ValueError, "Duplicate"):
            index_entries({"schema": "robodyna.source_transformations.v1", "files": [self.entry] * 2})
        self.entry["canonical_path"] = "../outside.cpp"
        with self.assertRaisesRegex(ValueError, "relative"):
            original_bytes(self.root, self.entry)


if __name__ == "__main__":
    unittest.main()
