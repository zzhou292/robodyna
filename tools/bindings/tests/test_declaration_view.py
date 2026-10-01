import hashlib
import json
from pathlib import Path
import tempfile
import unittest

from tools.bindings import declaration_view as module


class DeclarationViewTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.original = b"class Old { public: double Value() { return 7; } };\n"
        (self.root / "Canonical.h").write_bytes(self.original.replace(b"Old", b"New"))
        (self.root / "Legacy.h").write_text("using Old = New;\n")
        self.entry = {"original_path": "Legacy.h", "canonical_path": "Canonical.h",
                      "original_sha256": hashlib.sha256(self.original).hexdigest(),
                      "forwarding_sha256": hashlib.sha256((self.root / "Legacy.h").read_bytes()).hexdigest(),
                      "replacements": [{"before": "Old", "after": "New", "count": 1, "code_token": True}]}
        self.ledger = self.root / "ledger.json"
        self.ledger.write_text(json.dumps({"schema": "robodyna.source_transformations.v1", "files": [self.entry]}))
        self.ledger_hash = hashlib.sha256(self.ledger.read_bytes()).hexdigest()
        self.output = self.root / "Generated.h"

    def invoke(self):
        return module.generate(self.root, self.ledger, "Legacy.h", self.entry["original_sha256"],
                               self.ledger_hash, self.output)

    def test_generated_view_is_exact_original_with_a_compiler_rejection_guard(self):
        receipt = self.invoke()
        self.assertTrue(self.output.read_bytes().endswith(self.original))
        self.assertIn(b"#ifndef SWIG\n#error", self.output.read_bytes())
        self.assertEqual(receipt["original_sha256"], hashlib.sha256(self.original).hexdigest())
        self.assertEqual(receipt["view_sha256"], hashlib.sha256(self.output.read_bytes()).hexdigest())
        with self.assertRaises(FileExistsError):
            self.invoke()

    def test_changed_public_api_or_arithmetic_cannot_hide_behind_the_view(self):
        for change in (b"return 8", b"return 7; } double Added() { return 9"):
            (self.root / "Canonical.h").write_bytes(self.original.replace(b"Old", b"New").replace(b"return 7", change))
            with self.assertRaises(ValueError):
                self.invoke()
            self.assertFalse(self.output.exists())

    def test_edited_ledger_or_forwarder_rejects(self):
        original = self.ledger.read_bytes()
        self.ledger.write_bytes(original + b" ")
        with self.assertRaisesRegex(ValueError, "ledger pin"):
            self.invoke()
        self.ledger.write_bytes(original)
        (self.root / "Legacy.h").write_text("class Old {};\n")
        with self.assertRaisesRegex(ValueError, "forwarding"):
            self.invoke()


if __name__ == "__main__":
    unittest.main()
