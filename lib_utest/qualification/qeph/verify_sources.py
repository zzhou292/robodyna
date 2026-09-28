#!/usr/bin/env python3
"""Read-only check of Q3a/Q3b/Q3c port, tests, dependencies and native donor.

Reuses the owning native extraction/include verifier. No download, generation,
compiler, CUDA call or workspace mutation is performed.
"""
import hashlib
import json
from pathlib import Path
import runpy

MANIFEST_SHA256 = "59259ddb0c551aebd9d96f596cf56fbff8b8b11e1346955540c5a414b4b1cea2"


def verify():
    directory = Path(__file__).resolve().parent
    root = directory.parents[2]
    raw = (directory / "source-manifest.json").read_bytes()
    if hashlib.sha256(raw).hexdigest() != MANIFEST_SHA256:
        raise RuntimeError("Q3 source manifest changed")
    manifest = json.loads(raw)
    native = root / "lib_utest/qualification/native/qeph"
    native_report = runpy.run_path(str(native / "verify_sources.py"))["verify"]()
    if (native_report["revision"] != manifest["donor_revision"] or
            hashlib.sha256((native / "source-manifest.json").read_bytes()).hexdigest() !=
            manifest["native_source_manifest_sha256"]):
        raise RuntimeError("Q3 native reference mismatch")
    for stage in ("startup_baseline", "kinematics_baseline"):
        baseline = manifest[stage]["manifest"]
        baseline_bytes = (root / baseline["path"]).read_bytes()
        if hashlib.sha256(baseline_bytes).hexdigest() != baseline["sha256"]:
            raise RuntimeError("Historical Q3 manifest changed: " + stage)
    runpy.run_path(str(directory.parent / "qeph_private_trial/verify_sources.py"))["verify"]()
    runpy.run_path(str(directory.parent / "qeph_current_domain/verify_sources.py"))["verify"]()
    count = 0
    for group in ("ported_files", "shared_and_native_dependencies", "tests", "donor_routines"):
        for entry in manifest[group]:
            path = Path(entry["path"])
            if path.is_absolute() or ".." in path.parts:
                raise RuntimeError("Invalid Q3 source path")
            data = (root / path).read_bytes()
            if hashlib.sha256(data).hexdigest() != entry["sha256"]:
                raise RuntimeError("Q3 source changed: " + str(path))
            if "bytes" in entry and len(data) != entry["bytes"]:
                raise RuntimeError("Q3 source size changed: " + str(path))
            if "git_blob_sha1" in entry:
                blob = hashlib.sha1(b"blob " + str(len(data)).encode() + b"\0" + data).hexdigest()
                if blob != entry["git_blob_sha1"]:
                    raise RuntimeError("Q3 source Git identity changed: " + str(path))
            count += 1
    return {"status": "passed", "checked_records": count, "revision": manifest["donor_revision"],
            "native_reference": native_report}


if __name__ == "__main__":
    print(json.dumps(verify(), sort_keys=True))
