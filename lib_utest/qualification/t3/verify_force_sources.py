#!/usr/bin/env python3
"""Read-only force-stage provenance; the owning startup verifier checks closure."""
import hashlib
import json
from pathlib import Path
import runpy

MANIFEST_SHA256 = "463c370191759778ff07e8a4342f448d9bc6e01cb7789a9f24fc5149dc2d1842"


def verify():
    directory = Path(__file__).resolve().parent
    root = directory.parents[2]
    raw = (directory / "force-source-manifest.json").read_bytes()
    if hashlib.sha256(raw).hexdigest() != MANIFEST_SHA256:
        raise RuntimeError("T3 force-stage manifest changed")
    manifest = json.loads(raw)
    startup = runpy.run_path(str(directory / "verify_sources.py"))["verify"]()
    if startup["revision"] != manifest["donor_revision"]:
        raise RuntimeError("T3 force donor differs from qualified startup")
    count = 0
    for group in ("ported_files", "tests", "shared_and_native_dependencies", "donor_routines"):
        for entry in manifest[group]:
            path = Path(entry["path"])
            if path.is_absolute() or ".." in path.parts:
                raise RuntimeError("Unsafe T3 force provenance path")
            data = (root / path).read_bytes()
            if len(data) != entry["bytes"] or hashlib.sha256(data).hexdigest() != entry["sha256"]:
                raise RuntimeError("T3 force dependency changed: " + str(path))
            if "git_blob_sha1" in entry:
                blob = hashlib.sha1(b"blob " + str(len(data)).encode() + b"\0" + data).hexdigest()
                if blob != entry["git_blob_sha1"]:
                    raise RuntimeError("T3 force donor Git identity changed: " + str(path))
            count += 1
    return {"status": "passed", "checked_records": count,
            "revision": manifest["donor_revision"], "startup_and_native": startup,
            "force_port_execution": False}


if __name__ == "__main__":
    print(json.dumps(verify(), sort_keys=True))
