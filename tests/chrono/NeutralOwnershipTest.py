"""Authenticate import bytes, reviewed transformations and actual compile owners."""

import ast
import hashlib
import json
from pathlib import Path
import sys
import unittest

from tools.migration.source_transform import index_entries, original_bytes, relative_file

def assignments(path):
    return {
        node.targets[0].id: ast.literal_eval(node.value)
        for node in ast.parse(path.read_text()).body
        if isinstance(node, ast.Assign) and len(node.targets) == 1
    }


class NeutralOwnership(unittest.TestCase):
    def test_source_bytes_and_partition(self):
        owners = assignments(NEUTRAL)
        inventory = assignments(NATIVE)["NATIVE_SOURCE_GROUPS"]
        document = json.loads(MANIFEST.read_text())
        transformation_document = json.loads(TRANSFORMATIONS.read_text())
        transformations = index_entries(transformation_document)
        workspace = TRANSFORMATIONS.parent.parent.parent
        all_sources = [path for group in inventory.values() for path in group]
        extracted = [path for group in owners["NEUTRAL_SOURCES"].values() for path in group]
        self.assertEqual(len(extracted), 21)
        self.assertEqual(len(extracted), len(set(extracted)))
        self.assertTrue(set(extracted).issubset(all_sources))
        remaining = [path for path in all_sources if path not in extracted]
        self.assertEqual(len(remaining), 461)
        self.assertEqual(set(remaining) | set(extracted), set(all_sources))
        self.assertFalse(set(remaining) & set(extracted))
        for name, component in document["components"].items():
            self.assertEqual(component["target"], owners["NEUTRAL_TARGETS"][name])
            self.assertEqual(set(component["sources"]), set(owners["NEUTRAL_SOURCES"][name]))
            self.assertEqual(set(component["headers"]), set(owners["NEUTRAL_HEADERS"][name]))
            for path, digest in (component["sources"] | component["headers"]).items():
                original = "src/compatibility/chrono/" + path
                if original in transformations:
                    entry = transformations[original]
                    self.assertEqual(entry["original_sha256"], digest)
                    contents = original_bytes(workspace, entry)
                else:
                    contents = (SOURCE / path).read_bytes()
                self.assertEqual(hashlib.sha256(contents).hexdigest(), digest, path)
        for original, label in owners["NEUTRAL_SOURCE_RELOCATIONS"].items():
            entry = transformations["src/compatibility/chrono/" + original]
            self.assertEqual(label.removeprefix("//").replace(":", "/"), entry["canonical_path"])
        for entry in transformations.values():
            original_bytes(workspace, entry)
        additions = transformation_document.get("added_sources", [])
        self.assertEqual(len(additions), len({entry["path"] for entry in additions}))
        for entry in additions:
            self.assertEqual(hashlib.sha256(relative_file(workspace, entry["path"]).read_bytes()).hexdigest(),
                             entry["sha256"], entry["path"])


if __name__ == "__main__":
    MANIFEST, NEUTRAL, NATIVE, ANCHOR, TRANSFORMATIONS = [Path(arg) for arg in sys.argv[1:6]]
    SOURCE = ANCHOR.parent
    del sys.argv[1:6]
    unittest.main()
