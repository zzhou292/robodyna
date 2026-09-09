#!/usr/bin/env python3
"""Verify pinned originals, unchanged routine extracts and selected leaf closure.

Read only: no fetch, compiler, preprocessing, source generation or solver run.
Full startup/material dispatch drivers are retained context, not closure roots.
"""
import hashlib
import json
from pathlib import Path
import re

REVISION = "a62b27e6baa555d222a580d6218867d0be4d70b5"
MANIFEST_SHA256 = "3f4891f6dab344de9c0c579b699e43e11b30a7359358170833395481e62c2e9f"
INCLUDE = re.compile(r'^\s*#\s*include\s*["<]([^">]+)[">]', re.MULTILINE)
CALL = re.compile(r'^\s*CALL\s+(\w+)', re.MULTILINE | re.IGNORECASE)
USE = re.compile(r'^\s*USE\s+(\w+)', re.MULTILINE | re.IGNORECASE)
MODULES = {"constant_mod", "precision_mod", "element_mod", "elbufdef_mod"}
INCLUDE_ORDER = ("engine/share/spe_inc", "engine/share/includes",
                 "engine/share/r8", "starter/share/includes")


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def verify():
    root = Path(__file__).resolve().parent
    raw = (root / "source-manifest.json").read_bytes()
    require(hashlib.sha256(raw).hexdigest() == MANIFEST_SHA256, "T3 manifest changed")
    manifest = json.loads(raw)
    require(manifest["revision"] == REVISION, "T3 source revision changed")
    entries = {entry["source"]: entry for entry in manifest["sources"]}
    require(len(entries) == len(manifest["sources"]), "Duplicate T3 source")
    total = 0
    for source, entry in entries.items():
        path = (root / entry["path"]).resolve()
        expected = ((root.parent / "qeph/original" if entry["shared_qeph_original"]
                     else root / "original") / source).resolve()
        require(path == expected, "Unexpected source ownership: " + source)
        data = path.read_bytes()
        blob = hashlib.sha1(b"blob " + str(len(data)).encode() + b"\0" + data).hexdigest()
        require(len(data) == entry["bytes"] and hashlib.sha256(data).hexdigest() == entry["sha256"]
                and blob == entry["git_blob_sha1"], "Pinned T3 source changed: " + source)
        total += len(data)
    require(total == manifest["source_bytes"], "T3 byte inventory changed")

    pending = []
    routines = {entry["routine"].upper() for entry in manifest["extractions"]}
    for entry in manifest["extractions"]:
        lines = (root / entries[entry["source"]]["path"]).read_bytes().splitlines(keepends=True)
        first, last = entry["first_line"], entry["last_line"]
        require(re.match(rb'\s*SUBROUTINE\s+' + entry["routine"].encode() + rb'\s*\(',
                         lines[first-1], re.IGNORECASE) and
                re.fullmatch(rb'\s*END\s*', lines[last-1], re.IGNORECASE),
                "Incomplete native routine: " + entry["path"])
        notice = (b"\n! Complete pinned routines, extracted without arithmetic edits.\n"
                  if entry["shared_qeph_extract"] else
                  b"\n! Complete pinned routine, extracted without arithmetic edits.\n")
        expected = b"".join(lines[:23]) + notice + b"".join(lines[first-1:last])
        data = (root / entry["path"]).read_bytes()
        require(data == expected and hashlib.sha256(data).hexdigest() == entry["sha256"],
                "Complete T3 extraction changed: " + entry["path"])
        text = data.decode("latin1")
        require(set(name.upper() for name in CALL.findall(text)) <= routines,
                "Missing complete native CALL target: " + entry["path"])
        require(set(name.lower() for name in USE.findall(text)) <= MODULES,
                "Missing complete native module: " + entry["path"])
        pending.append((entry["source"], text))
    pending += [(source, (root / entry["path"]).read_text(encoding="latin1"))
                for source, entry in entries.items() if source.startswith("common_source/modules/")]
    visited = set()
    while pending:
        source, text = pending.pop()
        # Two selected routines in c3evec3 use the same literal includes. Each
        # routine's calls were independently checked above before deduplication.
        if source in visited:
            continue
        visited.add(source)
        # Exact nonselected CPP_ppc branch retained verbatim in comlock.inc.
        # The frozen GNU CPP_p4linux964 / no-_OPENMP profile never USEs it.
        platform_only={"f_pthread"} if source=="engine/share/spe_inc/comlock.inc" else set()
        require(set(name.lower() for name in USE.findall(text)) <= MODULES | platform_only,
                "Missing native module closure: " + source)
        for name in INCLUDE.findall(text):
            choices = [str(Path(source).parent / name)] + [directory + "/" + name for directory in INCLUDE_ORDER]
            found = next((choice for choice in choices if choice in entries), None)
            require(found is not None, "Missing pinned include " + name + " from " + source)
            pending.append((found, (root / entries[found]["path"]).read_text(encoding="latin1")))
    adapter = (root / "NativeT3Startup.F").read_text(encoding="latin1")
    require(set(name.upper() for name in CALL.findall(adapter)) == {"C3EVEC3"},
            "Startup native call boundary changed")
    require(set(INCLUDE.findall(adapter)) <= {"implicit_f.inc", "mvsiz_p.inc"},
            "Startup include boundary changed")
    return {"status": "passed", "revision": REVISION, "original_files": len(entries),
            "original_bytes": total, "complete_routine_extracts": len(manifest["extractions"]),
            "recursive_include_files": len(visited), "native_execution": False}


if __name__ == "__main__":
    print(json.dumps(verify(), sort_keys=True))
