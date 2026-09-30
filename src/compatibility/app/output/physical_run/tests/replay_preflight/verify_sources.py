#!/usr/bin/env python3
"""Frozen whole-file reversals plus unchanged reader/owner boundary evidence."""
import hashlib
import json
from pathlib import Path

def sha(value):
    return hashlib.sha256(value).hexdigest()

def main():
    directory = Path(__file__).resolve().parent
    root = directory.parents[3]
    manifest = json.loads((directory / "source-proof.json").read_text())
    for row in manifest["reversals"]:
        content = (root / row["path"]).read_bytes()
        assert sha(content) == row["candidate_sha256"], row["path"]
        restored = content.decode()
        for edit in reversed(row["edits"]):
            assert restored.count(edit["candidate"]) == 1, row["path"]
            restored = restored.replace(edit["candidate"], edit["baseline"], 1)
        assert sha(restored.encode()) == row["baseline_sha256"], row["path"]
    for family in ("unchanged", "new_production"):
        for path, expected in manifest[family].items():
            assert sha((root / path).read_bytes()) == expected, path
    print("PASS: 9 complete source/caller reversals, 14 unchanged owner/reader files, new preflight pin")

if __name__ == "__main__":
    main()
