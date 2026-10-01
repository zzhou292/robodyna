import hashlib
import json
from pathlib import Path
import tempfile
import unittest

from tools.migration.verify_branding import verify


def sha(data):
    return hashlib.sha256(data).hexdigest()


class BrandingVerificationTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        self.path = "src/compatibility/chrono/demo.cpp"
        source = self.root / self.path
        source.parent.mkdir(parents=True)
        before = b'// notice\nwindow("Old brand");\n'
        after = before.replace(b'Old brand', b'New brand')
        source.write_bytes(after)
        entry = dict(original_path=self.path, canonical_path=self.path, original_sha256=sha(before),
                     replacements=[dict(before='"Old brand"', after='"New brand"', count=1)])
        self.ledger = self.root / "ledger.json"
        self.ledger.write_text(json.dumps(dict(schema="robodyna.source_transformations.v1", files=[entry])))
        for name in ("master.png", "runtime.png", "historical.png"):
            (self.root / name).write_bytes(name.encode() if name == "historical.png" else b"approved asset bytes")
        self.document = dict(
            schema="robodyna.branding_replacements.v1",
            changed_text=[dict(path=self.path, before_sha256=sha(before), after_sha256=sha(after),
                               transformation_original_path=self.path, prior_replacement_count=0)],
            retired_assets=[dict(path="old-logo.png")],
            approved_generated_master=dict(path="master.png", sha256=sha(b"approved asset bytes")),
            byte_identical_replacements=[dict(path="runtime.png", sha256=sha(b"approved asset bytes"))],
            preserved_historical_screenshot=dict(path="historical.png", sha256=sha(b"historical.png")),
            source_proof=dict(imported_changed_text_files=1),
        )
        self.manifest = self.root / "manifest.json"

    def check(self):
        self.manifest.write_text(json.dumps(self.document))
        return verify(self.root, self.manifest, self.ledger)

    def test_real_file_and_inverse_verification_pass(self):
        self.assertTrue(self.check()["passed"])

    def test_retired_asset_reappearance_and_broken_symlink_reject(self):
        old = self.root / "old-logo.png"
        old.write_bytes(b"retired")
        with self.assertRaisesRegex(ValueError, "Retired"):
            self.check()
        old.unlink()
        old.symlink_to("missing")
        with self.assertRaisesRegex(ValueError, "Retired"):
            self.check()

    def test_changed_source_copy_or_preserved_evidence_reject(self):
        for name in (self.path, "runtime.png", "historical.png"):
            with self.subTest(name=name):
                path = self.root / name
                original = path.read_bytes()
                path.write_bytes(original + b"modified")
                with self.assertRaisesRegex(ValueError, "identity"):
                    self.check()
                path.write_bytes(original)

    def test_missing_inverse_and_wrong_preimage_reject(self):
        row = self.document["changed_text"][0]
        key = row.pop("transformation_original_path")
        with self.assertRaisesRegex(ValueError, "inverse history"):
            self.check()
        row["transformation_original_path"] = key
        row["before_sha256"] = "0" * 64
        with self.assertRaisesRegex(ValueError, "beyond"):
            self.check()

    def test_retirement_path_escape_rejects(self):
        self.document["retired_assets"][0]["path"] = "../outside.png"
        with self.assertRaisesRegex(ValueError, "relative"):
            self.check()


if __name__ == "__main__":
    unittest.main()
