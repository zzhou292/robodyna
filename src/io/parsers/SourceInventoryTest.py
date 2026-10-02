"""Authenticate original parser inputs and unique implementation ownership."""
import ast
import hashlib
import json
from pathlib import Path
import re
import sys
import unittest

BASELINE = Path(sys.argv.pop(1))
INPUTS = Path(sys.argv.pop(1))
BUILD = Path(sys.argv.pop(1))
ROOT = INPUTS.parents[2]


class ParserSources(unittest.TestCase):
    def test_retained_source_bytes(self):
        for entry in json.loads(BASELINE.read_text())["files"]:
            path = ROOT / entry["path"]
            with self.subTest(path=entry["path"]):
                self.assertEqual(hashlib.sha256(path.read_bytes()).hexdigest(), entry["sha256"])

    def test_every_module_translation_unit_has_one_owner(self):
        expected = [entry["path"].removeprefix("src/chrono_parsers/")
                    for entry in json.loads(BASELINE.read_text())["files"] if entry["path"].endswith(".cpp")]
        actual = re.findall(r'_SOURCE \+ "([^"\n]+\.cpp)"', BUILD.read_text())
        self.assertEqual(len(expected), 13)
        self.assertCountEqual(actual, expected)
        self.assertEqual(len(actual), len(set(actual)))


if __name__ == "__main__":
    unittest.main()
