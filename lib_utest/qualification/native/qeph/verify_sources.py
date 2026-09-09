#!/usr/bin/env python3
"""Read-only check of pinned originals and complete unedited Q1 extracts.

Uses the existing native/phase verifier's SHA256 + Git-blob convention. No
download, code generation, compiler invocation or source mutation occurs here.
"""
import hashlib
import json
from pathlib import Path

REVISION = "a62b27e6baa555d222a580d6218867d0be4d70b5"
MANIFEST_SHA256 = "eacae6570282ec84b6811ab0704f77d30cb095a0d3f7a95d7dfd530d1d6cebc9"


def verify():
    root = Path(__file__).resolve().parent
    raw = (root / "source-manifest.json").read_bytes()
    if hashlib.sha256(raw).hexdigest() != MANIFEST_SHA256:
        raise RuntimeError("Q1 source manifest changed")
    manifest = json.loads(raw)
    if manifest["revision"] != REVISION or len(manifest["sources"]) != 27:
        raise RuntimeError("Q1 revision or closure mismatch")
    for entry in manifest["sources"]:
        data = (root / "original" / entry["path"]).read_bytes()
        blob = hashlib.sha1(b"blob " + str(len(data)).encode() + b"\0" + data).hexdigest()
        if (len(data) != entry["bytes"] or hashlib.sha256(data).hexdigest() != entry["sha256"]
                or blob != entry["git_blob_sha1"]):
            raise RuntimeError("Q1 original changed: " + entry["path"])
    for entry in manifest["extractions"]:
        source = (root / "original" / entry["source"]).read_bytes().splitlines(keepends=True)
        expected = b"".join(source[:23]) + b"\n! Complete pinned routines, extracted without arithmetic edits.\n"
        for routine in entry["ranges"]:
            expected += b"".join(source[routine["first_line"]-1:routine["last_line"]])
        actual = (root / entry["path"]).read_bytes()
        if actual != expected or hashlib.sha256(actual).hexdigest() != entry["sha256"]:
            raise RuntimeError("Q1 complete extraction changed: " + entry["path"])
    return {"status": "passed", "revision": REVISION, "original_files": 27,
            "complete_extraction_units": len(manifest["extractions"])}


if __name__ == "__main__":
    print(json.dumps(verify(), sort_keys=True))
