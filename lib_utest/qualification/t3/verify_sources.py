#!/usr/bin/env python3
"""Read-only T3 port/source provenance check; reuses the native closure verifier."""
import hashlib
import json
from pathlib import Path
import runpy

MANIFEST_SHA256 = "87b128d30d3af618d86a6c0b3c26578303c6b34ee9a7e5068b4c90486798a8cb"


def verify():
    directory = Path(__file__).resolve().parent
    root = directory.parents[2]
    data = (directory / "source-manifest.json").read_bytes()
    if hashlib.sha256(data).hexdigest() != MANIFEST_SHA256:
        raise RuntimeError("T3 port manifest changed")
    manifest = json.loads(data)
    native = root / "lib_utest/qualification/native/t3"
    report = runpy.run_path(str(native / "verify_sources.py"))["verify"]()
    if report["revision"] != manifest["donor_revision"] or hashlib.sha256(
            (native / "source-manifest.json").read_bytes()).hexdigest() != manifest["native_source_manifest_sha256"]:
        raise RuntimeError("T3 native source identity changed")
    count = 0
    for group in ("ported_files", "tests", "shared_and_native_dependencies", "donor_routines"):
        for entry in manifest[group]:
            path = Path(entry["path"])
            if path.is_absolute() or ".." in path.parts:
                raise RuntimeError("Unsafe T3 provenance path")
            raw = (root / path).read_bytes()
            if len(raw) != entry["bytes"] or hashlib.sha256(raw).hexdigest() != entry["sha256"]:
                raise RuntimeError("T3 source changed: " + str(path))
            if "git_blob_sha1" in entry:
                blob = hashlib.sha1(b"blob " + str(len(raw)).encode() + b"\0" + raw).hexdigest()
                if blob != entry["git_blob_sha1"]:
                    raise RuntimeError("T3 donor Git identity changed: " + str(path))
            count += 1
    return {"status": "passed", "checked_records": count, "revision": report["revision"],
            "native_reference": report, "port_execution": False}


if __name__ == "__main__":
    print(json.dumps(verify(), sort_keys=True))
