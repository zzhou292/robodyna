#!/usr/bin/env python3
"""Read-only check of pinned originals and complete unedited Q1/Q2 extracts.

Uses the existing native/phase verifier's SHA256 + Git-blob convention. No
download, code generation, compiler invocation or source mutation occurs here.
"""
import hashlib
import json
from pathlib import Path
import re

REVISION = "a62b27e6baa555d222a580d6218867d0be4d70b5"
MANIFEST_SHA256 = "46d2759036e16558500c91a9d14dd225f203aa5d2abec81bb951ac1b956030be"


NATIVE_ADAPTERS = (
    "NativeQephStartup.F", "NativeQephKinematics.F", "QephNativeGeometry.F",
    "QephNativeHistory.F", "QephNativeMaterial.F", "QephNativeLaw1.F",
    "QephNativeStiffness.F", "NativeQephForce.F", "NativeQephScatter.F",
)
INCLUDE = re.compile(r'^\s*#\s*include\s*["<]([^">]+)[">]', re.MULTILINE)


def verify_include_closure(root, manifest):
    """Check recursive literal includes of compiled units, including arch branches.

    Follow the owning CMake include search order. A found donor include must be
    pinned in the manifest; unrelated full original material dispatch is not a
    compiled root. This performs no preprocessing or compiler invocation.
    """
    original = root / "original"
    directories = [original / relative for relative in (
        "engine/share/spe_inc", "engine/share/includes", "engine/share/r8", "starter/share/includes")]
    retained = {original / entry["path"] for entry in manifest["sources"]}
    pending = [root / path for path in NATIVE_ADAPTERS]
    pending += [root / entry["path"] for entry in manifest["extractions"]]
    pending += [path for path in retained if "common_source/modules/" in str(path)]
    visited = set()
    while pending:
        path = pending.pop()
        if path in visited:
            continue
        visited.add(path)
        for name in INCLUDE.findall(path.read_text(encoding="latin1")):
            found = next((directory / name for directory in [path.parent] + directories
                          if (directory / name).is_file()), None)
            if found is None:
                raise RuntimeError("Missing native include " + name + " from " + str(path.relative_to(root)))
            if found not in retained:
                raise RuntimeError("Unpinned native include: " + str(found.relative_to(root)))
            pending.append(found)
    return len(visited)


def verify():
    root = Path(__file__).resolve().parent
    raw = (root / "source-manifest.json").read_bytes()
    if hashlib.sha256(raw).hexdigest() != MANIFEST_SHA256:
        raise RuntimeError("Q1 source manifest changed")
    manifest = json.loads(raw)
    if manifest["revision"] != REVISION or len(manifest["sources"]) != 55:
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
    visited = verify_include_closure(root, manifest)
    return {"recursive_include_files": visited, "status": "passed", "revision": REVISION, "original_files": 55,
            "complete_extraction_units": len(manifest["extractions"])}


if __name__ == "__main__":
    print(json.dumps(verify(), sort_keys=True))
