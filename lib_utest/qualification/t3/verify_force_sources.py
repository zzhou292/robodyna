#!/usr/bin/env python3
"""Read-only force-stage provenance; the owning startup verifier checks closure."""
import hashlib
import json
from pathlib import Path
import runpy

MANIFEST_SHA256 = "cf1831c0b3131fb1a84cab8a2adc8b78581ef84c8ad2c5d7df3ad9affccff9b2"


# Reviewed test-tooling transition 670452fa only. Numerical/native/test-body
# records remain exact, and the frozen force manifest itself is unchanged.
STARTUP_VERIFIER_ORIGINAL = (2001, "1a58ad357485fad4ed3863ad1b7d4d09e7d4fa1bb01af47577272bb828db6d6f")
STARTUP_VERIFIER_REVIEWED = (3667, "43fd87e6346a26488f19844ab61be6e9f693165fc8e00bdaf97cfe83cb290080")


def reviewed_startup_tooling(entry, data):
    actual = (len(data), hashlib.sha256(data).hexdigest())
    if actual != STARTUP_VERIFIER_REVIEWED:
        return False
    if (entry["bytes"], entry["sha256"]) != STARTUP_VERIFIER_ORIGINAL:
        raise RuntimeError("Frozen T3 startup verifier provenance changed")
    return True


def verify():
    directory = Path(__file__).resolve().parent
    root = directory.parents[2]
    raw = (directory / "force-source-manifest.json").read_bytes()
    if hashlib.sha256(raw).hexdigest() != MANIFEST_SHA256:
        raise RuntimeError("T3 force-stage manifest changed")
    manifest = json.loads(raw)
    startup_module = runpy.run_path(str(directory / "verify_sources.py"))
    startup = startup_module["verify"]()
    if startup["revision"] != manifest["donor_revision"]:
        raise RuntimeError("T3 force donor differs from qualified startup")
    count = 0
    tooling_transition = False
    for group in ("ported_files", "tests", "shared_and_native_dependencies", "donor_routines"):
        for entry in manifest[group]:
            path = Path(entry["path"])
            if path.is_absolute() or ".." in path.parts:
                raise RuntimeError("Unsafe T3 force provenance path")
            data = (root / path).read_bytes()
            if str(path) == "lib_src/elements/t3/BUILD.bazel":
                data = startup_module["frozen_build_registration"](data)
            tooling = str(path) == "lib_utest/qualification/t3/verify_sources.py" and reviewed_startup_tooling(entry, data)
            tooling_transition = tooling_transition or tooling
            if not tooling and (len(data) != entry["bytes"] or hashlib.sha256(data).hexdigest() != entry["sha256"]):
                raise RuntimeError("T3 force dependency changed: " + str(path))
            if "git_blob_sha1" in entry:
                blob = hashlib.sha1(b"blob " + str(len(data)).encode() + b"\0" + data).hexdigest()
                if blob != entry["git_blob_sha1"]:
                    raise RuntimeError("T3 force donor Git identity changed: " + str(path))
            count += 1
    return {"status": "passed", "checked_records": count,
            "revision": manifest["donor_revision"], "startup_and_native": startup,
            "force_port_execution": False,
            "reviewed_startup_verifier_transition": tooling_transition}


if __name__ == "__main__":
    print(json.dumps(verify(), sort_keys=True))
