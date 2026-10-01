"""Only exact, pinned parser-view source locations may be remapped."""

import hashlib
import json
from pathlib import Path
import tempfile
import unittest

from tools.bindings.diagnostic_origins import GUARD_PREFIX, authenticated_views, map_diagnostics


def digest(data):
    return hashlib.sha256(data).hexdigest()


class DiagnosticOrigins(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        root = Path(self.temp.name)
        self.old, self.new = root / "baseline", root / "candidate"
        self.original = "legacy/System.h"
        self.output = "robodyna_swig/SystemDeclarations.h"
        original = b"// Original declaration\nnamespace old {\nclass Visual;\n}\n"
        self.source = self.old / self.original
        self.source.parent.mkdir(parents=True)
        self.source.write_bytes(original)
        self.view = self.new / "swig_generated" / self.output
        self.view.parent.mkdir(parents=True)
        self.view.write_bytes(GUARD_PREFIX + original)
        self.receipt = {"schema": "robodyna.swig_declaration_view.v1", "original_path": self.original,
                        "original_sha256": digest(original), "view_sha256": digest(self.view.read_bytes())}
        (self.old / "parser-inputs.json").write_text(json.dumps({"source_inputs": {self.original: digest(original)}}))
        self.save_receipt()

    def save_receipt(self):
        self.view.with_suffix(".h.json").write_text(json.dumps(self.receipt))
        (self.new / "parser-inputs.json").write_text(json.dumps({"generated_declaration_views": {self.output: self.receipt}}))

    def test_exact_origin_mapping_preserves_warning_content(self):
        origins = authenticated_views(self.new, self.old)
        old = f"{self.source}:3: Warning 402: incomplete base\n"
        new = f"{self.view}:7: Warning 402: incomplete base\n"
        self.assertEqual(map_diagnostics(old, self.old), map_diagnostics(new, self.new, origins))
        changed = new.replace("402: incomplete", "401: unknown")
        self.assertNotEqual(map_diagnostics(old, self.old), map_diagnostics(changed, self.new, origins))

    def test_altered_historical_bytes_fail(self):
        self.source.write_bytes(self.source.read_bytes() + b"// changed\n")
        with self.assertRaisesRegex(ValueError, "hash changed"):
            authenticated_views(self.new, self.old)

    def test_altered_view_bytes_fail(self):
        self.view.write_bytes(self.view.read_bytes() + b"// changed\n")
        with self.assertRaisesRegex(ValueError, "hash changed"):
            authenticated_views(self.new, self.old)

    def test_rehashed_wrong_guard_still_fails(self):
        self.view.write_bytes(self.view.read_bytes().replace(b"#ifndef SWIG", b"#ifdef SWIG"))
        self.receipt["view_sha256"] = digest(self.view.read_bytes())
        self.save_receipt()
        with self.assertRaisesRegex(ValueError, "exact guard prefix"):
            authenticated_views(self.new, self.old)

    def test_receipt_drift_and_outside_line_numbers_fail(self):
        path = self.view.with_suffix(".h.json")
        altered = dict(self.receipt, original_sha256="0" * 64)
        path.write_text(json.dumps(altered))
        with self.assertRaisesRegex(ValueError, "receipt differs"):
            authenticated_views(self.new, self.old)
        self.save_receipt()
        origins = authenticated_views(self.new, self.old)
        for number in (1, 100):
            with self.assertRaisesRegex(ValueError, "outside"):
                map_diagnostics(f"{self.view}:{number}: Warning 402: incomplete\n", self.new, origins)


if __name__ == "__main__":
    unittest.main()
