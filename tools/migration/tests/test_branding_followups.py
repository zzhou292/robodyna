"""Reject repinned history and unreviewed edits while admitting exact suffixes."""
import hashlib
import json
from pathlib import Path
import tempfile
import unittest

from tools.migration.branding_followups import object_sha256
from tools.migration.verify_branding import verify


def sha(data):
    return hashlib.sha256(data).hexdigest()


class BrandingFollowupsTest(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        self.name = "src/compatibility/chrono/demo.cpp"
        self.source = self.root / self.name
        self.source.parent.mkdir(parents=True)
        self.original = b'// notice\n#include "Old.h"\nwindow("Old brand");\n'
        self.branded = self.original.replace(b'Old brand', b'Robodyna')
        self.current = self.branded.replace(b'Old.h', b'Correct.h')
        self.source.write_bytes(self.current)
        branding = dict(before='"Old brand"', after='"Robodyna"', count=1)
        suffix = dict(before='"Old.h"', after='"Correct.h"', count=1)
        historical_entry = dict(original_path=self.name, canonical_path=self.name,
                                original_sha256=sha(self.original), replacements=[branding])
        self.entry = dict(historical_entry, replacements=[branding, suffix])
        self.ledger = self.root / "ledger.json"
        self.manifest = self.root / "manifest.json"
        self.registry = self.root / "followups.json"
        (self.root / "master.png").write_bytes(b"approved master")
        (self.root / "historical.png").write_bytes(b"historical screenshot")
        self.document = dict(
            schema="robodyna.branding_replacements.v1",
            changed_text=[dict(path=self.name, before_sha256=sha(self.original), after_sha256=sha(self.branded),
                               transformation_original_path=self.name, prior_replacement_count=0)],
            source_proof=dict(imported_changed_text_files=1), retired_assets=[], byte_identical_replacements=[],
            approved_generated_master=dict(path="master.png", sha256=sha(b"approved master")),
            preserved_historical_screenshot=dict(path="historical.png", sha256=sha(b"historical screenshot")))
        self.manifest.write_text(json.dumps(self.document))
        self.followups = dict(
            schema="robodyna.branding_followups.v1", historical_checkpoint="a" * 40,
            historical_manifest_sha256=sha(self.manifest.read_bytes()),
            files=[dict(path=self.name, kind="imported_source", original_path=self.name,
                        historical_sha256=sha(self.branded), current_sha256=sha(self.current),
                        historical_replacement_count=1, historical_entry_sha256=object_sha256(historical_entry))])

    def check(self, followups=True):
        self.ledger.write_text(json.dumps(dict(schema="robodyna.source_transformations.v1", files=[self.entry])))
        self.registry.write_text(json.dumps(self.followups))
        return verify(self.root, self.manifest, self.ledger, self.registry if followups else None)

    def test_exact_later_include_edit_restores_both_historical_boundaries(self):
        result = self.check()
        self.assertTrue(result["passed"])
        self.assertEqual(result["reviewed_followup_files"], 1)
        self.assertEqual(result["imported_source_histories"], 1)

    def test_current_edit_is_not_admitted_without_a_followup(self):
        with self.assertRaisesRegex(ValueError, "identity"):
            self.check(False)

    def test_historical_mode_still_accepts_original_branded_checkpoint(self):
        self.source.write_bytes(self.branded)
        self.entry["replacements"] = self.entry["replacements"][:1]
        self.assertEqual(self.check(False)["reviewed_followup_files"], 0)

    def test_repinning_historical_manifest_is_rejected(self):
        self.document["changed_text"][0]["after_sha256"] = sha(self.current)
        self.manifest.write_text(json.dumps(self.document))
        with self.assertRaisesRegex(ValueError, "Historical branding manifest"):
            self.check()

    def test_rewriting_source_history_prefix_is_rejected(self):
        self.entry["original_sha256"] = "0" * 64
        with self.assertRaisesRegex(ValueError, "Historical source history"):
            self.check()

    def test_repinning_current_digest_does_not_hide_unreviewed_text(self):
        changed = self.current + b"unreviewed extra code\n"
        self.source.write_bytes(changed)
        self.followups["files"][0]["current_sha256"] = sha(changed)
        with self.assertRaisesRegex(ValueError, "beyond"):
            self.check()

    def test_owned_build_metadata_requires_an_exact_inverse(self):
        owned = "src/compatibility/chrono/BUILD.bazel"
        self.source.rename(self.root / owned)
        self.document["changed_text"][0].update(path=owned)
        self.document["changed_text"][0].pop("transformation_original_path")
        self.document["source_proof"]["imported_changed_text_files"] = 0
        self.manifest.write_text(json.dumps(self.document))
        self.followups["historical_manifest_sha256"] = sha(self.manifest.read_bytes())
        self.followups["files"] = [dict(path=owned, kind="owned_text", historical_sha256=sha(self.branded),
                                        current_sha256=sha(self.current), replacements=[self.entry["replacements"][1]])]
        self.assertTrue(self.check()["passed"])
        self.followups["files"][0]["replacements"] = []
        with self.assertRaisesRegex(ValueError, "exact inverse"):
            self.check()

    def test_duplicate_unlisted_and_symlink_paths_are_rejected(self):
        row = self.followups["files"][0]
        self.followups["files"].append(dict(row))
        with self.assertRaisesRegex(ValueError, "Duplicate"):
            self.check()
        self.followups["files"] = [dict(row, path="unlisted.cpp")]
        with self.assertRaisesRegex(ValueError, "unlisted"):
            self.check()
        self.followups["files"] = [row]
        self.source.unlink()
        self.source.symlink_to(self.root / "master.png")
        with self.assertRaisesRegex(ValueError, "regular"):
            self.check()


if __name__ == "__main__":
    unittest.main()
