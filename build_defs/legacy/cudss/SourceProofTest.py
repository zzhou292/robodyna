"""Prove that only the reviewed cuDSS call boundary changed in Newton."""

import hashlib
import json
from pathlib import Path
import sys
import tempfile
import unittest

from tools.migration.source_transform import index_entries, original_bytes


BASELINE_SHA256 = "49b666e4f8a2b2df0cb436db3ac109c44a98dd14c93bf665b79faa7f4803f928"
ROOT = Path(__file__).resolve().parents[3]
LEDGER = ROOT / "build_defs/legacy/cudss/SOURCE_TRANSFORMATIONS.json"
SOURCE = ROOT / "src/fea/legacy/lib_src/solvers/SyncedNewton.cu"


class SourceProofTest(unittest.TestCase):
    def setUp(self):
        document = json.loads(LEDGER.read_text())
        self.assertEqual(document["qualified_commit"],
                         "f0cdeffaef85ea1f97c2162790dbd091fb2e4853")
        entries = index_entries(document)
        self.assertEqual(len(entries), 1)
        self.entry = next(iter(entries.values()))
        self.assertEqual(self.entry["original_sha256"], BASELINE_SHA256)
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.root = Path(self.directory.name)
        self.source = self.root / self.entry["canonical_path"]
        self.source.parent.mkdir(parents=True)
        self.source.write_bytes(SOURCE.read_bytes())

    def test_exact_inverse_preserves_entire_original_solver(self):
        restored = original_bytes(self.root, self.entry)
        self.assertEqual(hashlib.sha256(restored).hexdigest(), BASELINE_SHA256)

    def test_numeric_edit_is_rejected_instead_of_hidden_by_api_recipe(self):
        current = self.source.read_text()
        before = "const double armijo_c1          = 1e-4;"
        self.assertEqual(current.count(before), 1)
        self.source.write_text(current.replace(before,
                                               "const double armijo_c1          = 2e-4;"))
        with self.assertRaisesRegex(ValueError, "Source differs beyond"):
            original_bytes(self.root, self.entry)


if __name__ == "__main__":
    if len(sys.argv) > 1:
        LEDGER, SOURCE = (Path(value) for value in sys.argv[1:3])
        del sys.argv[1:3]
    unittest.main()
