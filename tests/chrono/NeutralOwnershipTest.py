"""Authenticate unchanged source bytes and the aggregate/neutral ownership split."""

import ast
import hashlib
import json
from pathlib import Path
import sys
import unittest


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
                self.assertEqual(hashlib.sha256((SOURCE / path).read_bytes()).hexdigest(), digest, path)


if __name__ == "__main__":
    MANIFEST, NEUTRAL, NATIVE, ANCHOR = [Path(arg) for arg in sys.argv[1:5]]
    SOURCE = ANCHOR.parent
    del sys.argv[1:5]
    unittest.main()
