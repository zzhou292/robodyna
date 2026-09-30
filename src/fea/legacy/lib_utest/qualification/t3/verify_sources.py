#!/usr/bin/env python3
"""Read-only T3 port/source provenance check; reuses the native closure verifier."""
import hashlib
import json
from pathlib import Path
import runpy

MANIFEST_SHA256 = "0cfcebdf3441428e28935cdfc3a8011df48bb9919042ecd25598e364acd8f9bf"


# The compact activity package adds only build registrations to this frozen
# startup/rates scope (reviewed compact commit b1f19ef7; frozen base 759e486a).
# Recognize that exact reviewed file, reverse its four
# additions, then retain the original manifest hash/size checks below.
COMPACT_ACTIVITY_BUILD_SHA256 = "94a0e9557b83d0a5fd43600de4fc006c4e52b67b86d037a8135a94d8445a9058"


def frozen_build_registration(raw):
    if hashlib.sha256(raw).hexdigest() != COMPACT_ACTIVITY_BUILD_SHA256:
        return raw
    text = raw.decode("utf-8")
    additions = (
        ('"mapped/Startup.cpp", "mapped/ActivityReport.cpp"', '"mapped/Startup.cpp"'),
        ('"mapped/AssemblyValues.h", "mapped/ActivityValues.h", "mapped/ActivityQuery.h"',
         '"mapped/AssemblyValues.h"'),
        ('"//lib_src/elements/mapped_shell:activity_layout", "//lib_src/elements/mapped_shell:observer_values"',
         '"//lib_src/elements/mapped_shell:observer_values"'),
        ('"mapped/Readback.cpp", "mapped/ActivityKernels.cu", "mapped/ActivityReadback.cu"',
         '"mapped/Readback.cpp"'),
    )
    for current, previous in additions:
        if text.count(current) != 1:
            raise RuntimeError("Compact T3 build registration changed")
        text = text.replace(current, previous, 1)
    return text.encode("utf-8")


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
    normalized_registration = False
    for group in ("ported_files", "tests", "shared_and_native_dependencies", "donor_routines"):
        for entry in manifest[group]:
            path = Path(entry["path"])
            if path.is_absolute() or ".." in path.parts:
                raise RuntimeError("Unsafe T3 provenance path")
            raw = (root / path).read_bytes()
            if str(path) == "lib_src/elements/t3/BUILD.bazel":
                normalized_registration = hashlib.sha256(raw).hexdigest() == COMPACT_ACTIVITY_BUILD_SHA256
                raw = frozen_build_registration(raw)
            if len(raw) != entry["bytes"] or hashlib.sha256(raw).hexdigest() != entry["sha256"]:
                raise RuntimeError("T3 source changed: " + str(path))
            if "git_blob_sha1" in entry:
                blob = hashlib.sha1(b"blob " + str(len(raw)).encode() + b"\0" + raw).hexdigest()
                if blob != entry["git_blob_sha1"]:
                    raise RuntimeError("T3 donor Git identity changed: " + str(path))
            count += 1
    return {"status": "passed", "checked_records": count, "revision": report["revision"],
            "native_reference": report, "port_execution": False,
            "normalized_compact_activity_build_registration": normalized_registration}


if __name__ == "__main__":
    print(json.dumps(verify(), sort_keys=True))
