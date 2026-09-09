#!/usr/bin/env python3
"""Read-only check of Q3a's declared port, tests, dependencies and native donor.

Reuses the owning native extraction/include verifier. No download, generation,
compiler, CUDA call or workspace mutation is performed.
"""
import hashlib
import json
from pathlib import Path
import runpy

MANIFEST_SHA256 = "84ab8306fb7ed9a063b28ac5bd5672d8cd5814cf1a04c60bf77dea53d01de2eb"


def verify():
    directory = Path(__file__).resolve().parent
    root = directory.parents[2]
    raw = (directory / "source-manifest.json").read_bytes()
    if hashlib.sha256(raw).hexdigest() != MANIFEST_SHA256:
        raise RuntimeError("Q3a source manifest changed")
    manifest = json.loads(raw)
    native = root / "lib_utest/qualification/native/qeph"
    native_report = runpy.run_path(str(native / "verify_sources.py"))["verify"]()
    if (native_report["revision"] != manifest["donor_revision"] or
            hashlib.sha256((native / "source-manifest.json").read_bytes()).hexdigest() !=
            manifest["native_source_manifest_sha256"]):
        raise RuntimeError("Q3a native reference mismatch")
    count = 0
    for group in ("ported_files", "shared_and_native_dependencies", "tests", "donor_routines"):
        for entry in manifest[group]:
            path = Path(entry["path"])
            if path.is_absolute() or ".." in path.parts:
                raise RuntimeError("Invalid Q3a source path")
            data = (root / path).read_bytes()
            if hashlib.sha256(data).hexdigest() != entry["sha256"]:
                raise RuntimeError("Q3a source changed: " + str(path))
            if "bytes" in entry and len(data) != entry["bytes"]:
                raise RuntimeError("Q3a source size changed: " + str(path))
            if "git_blob_sha1" in entry:
                blob = hashlib.sha1(b"blob " + str(len(data)).encode() + b"\0" + data).hexdigest()
                if blob != entry["git_blob_sha1"]:
                    raise RuntimeError("Q3a source Git identity changed: " + str(path))
            count += 1
    return {"status": "passed", "checked_records": count, "revision": manifest["donor_revision"],
            "native_reference": native_report}


if __name__ == "__main__":
    print(json.dumps(verify(), sort_keys=True))
