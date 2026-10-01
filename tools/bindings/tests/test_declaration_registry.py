"""Reject stale dependency paths and unreviewed parser inputs before generation."""

import hashlib
import json
from pathlib import Path
import tempfile
import unittest

from tools.bindings.declaration_registry import read


class DeclarationRegistry(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.directory = self.root / "build_defs/bindings"
        self.directory.mkdir(parents=True)
        entry = {"original_path": "legacy/Body.h", "canonical_path": "include/robodyna/mbd/RbBody.h",
                 "original_sha256": "a" * 64, "replacements": []}
        ledger = json.dumps({"schema": "robodyna.source_transformations.v1", "files": [entry]}).encode()
        (self.root / "ledger.json").write_bytes(ledger)
        contract = {"original_path": entry["original_path"], "expected_original_sha256": "a" * 64,
                    "expected_ledger_sha256": hashlib.sha256(ledger).hexdigest(), "output": "robodyna_swig/BodyDeclarations.h"}
        (self.directory / "body_view_contract.json").write_text(json.dumps(contract))
        self.document = {"schema": "robodyna.swig_declaration_views.v1", "views": [{
            "name": "body", "contract": "build_defs/bindings/body_view_contract.json", "ledger": "ledger.json",
            "canonical": entry["canonical_path"], "forwarder": entry["original_path"], "requires_fea": False}]}
        self.save()

    def save(self):
        (self.directory / "declaration_views.json").write_text(json.dumps(self.document))

    def test_declared_original_and_canonical_identity_are_admitted(self):
        self.assertEqual(read(self.root)[0]["output"], "robodyna_swig/BodyDeclarations.h")

    def test_wrong_canonical_dependency_is_rejected(self):
        self.document["views"][0]["canonical"] = "include/robodyna/Wrong.h"
        self.save()
        with self.assertRaisesRegex(ValueError, "canonical dependency"):
            read(self.root)

    def test_changed_ledger_bytes_are_rejected(self):
        with (self.root / "ledger.json").open("a") as stream:
            stream.write(" ")
        with self.assertRaisesRegex(ValueError, "reviewed pin"):
            read(self.root)

    def test_duplicate_or_outside_paths_are_rejected(self):
        self.document["views"].append(dict(self.document["views"][0]))
        self.save()
        with self.assertRaisesRegex(ValueError, "Duplicate"):
            read(self.root)
        self.document["views"] = self.document["views"][:1]
        self.document["views"][0]["ledger"] = "../outside.json"
        self.save()
        with self.assertRaisesRegex(ValueError, "repository-relative"):
            read(self.root)


if __name__ == "__main__":
    unittest.main()
