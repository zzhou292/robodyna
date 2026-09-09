#!/usr/bin/env python3
"""Acquire only explicitly selected pinned T3 sources; never build or run them.

Reuses the workspace's exact Git-blob/SHA256 acquisition convention. Existing
QEPH originals are borrowed by path rather than copied into a second closure.
Files publish only after verification; different existing bytes are rejected.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import urllib.request

REVISION = "a62b27e6baa555d222a580d6218867d0be4d70b5"
ROOT = Path(__file__).resolve().parent
WORKSPACE = ROOT.parents[4]
DONOR = Path("/tmp/chrono-yaris-plan/source-reference/OpenRadioss")
QEPH = ROOT.parent / "qeph/original"
MAX_FILE = 2 * 1024 * 1024
MAX_TOTAL = 16 * 1024 * 1024
ENV = dict(os.environ, GIT_NO_LAZY_FETCH="1")


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def git(*args, check=True):
    return subprocess.run(["git", "-C", str(DONOR), "-c", "remote.origin.promisor=false", *args],
                          env=ENV, check=check, stdout=subprocess.PIPE,
                          stderr=subprocess.PIPE, timeout=20)


def read_source(source, blob):
    destination = ROOT / "original" / source
    candidates = [(QEPH / source, "shared qualified QEPH original"),
                  (destination, "existing T3 original"), (DONOR / source, "local pinned checkout")]
    for base in ("openradioss-shell-execution-trace", "openradioss-qeph-law1-closure-1",
                 "openradioss-shell-context"):
        candidates.append((WORKSPACE / "crash-work/deps" / base / source, "retained workspace source"))
    for path, method in candidates:
        if path.is_file():
            require(path.stat().st_size <= MAX_FILE, "Source exceeds file cap: " + source)
            return path.read_bytes(), path if path.is_relative_to(QEPH) else destination, method
    result = git("cat-file", "blob", blob, check=False)
    if result.returncode == 0:
        return result.stdout, destination, "local pinned Git object"
    url = f"https://raw.githubusercontent.com/OpenRadioss/OpenRadioss/{REVISION}/{source}"
    with urllib.request.urlopen(url, timeout=20) as response:
        require(response.status == 200, "Unexpected source response")
        data = response.read(MAX_FILE + 1)
    return data, destination, "official pinned HTTPS"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("sources", nargs="+", help="Exact donor-relative file paths")
    args = parser.parse_args()
    require(git("rev-parse", "HEAD").stdout.decode().strip() == REVISION, "Donor revision differs")
    manifest_path = ROOT / "source-manifest.json"
    if manifest_path.exists():
        manifest = json.loads(manifest_path.read_text())
        require(manifest["revision"] == REVISION, "Existing T3 pin differs")
    else:
        manifest = {"schema": "tl.t3-native-source-inventory.v1", "revision": REVISION,
                    "origin": "https://github.com/OpenRadioss/OpenRadioss",
                    "scope": "Original sources and dependencies for review; no callable reference or closed build asserted",
                    "license": "AGPL-3.0-or-later", "sources": []}
    saved = {entry["source"]: entry for entry in manifest["sources"]}
    total = sum(entry["bytes"] for entry in saved.values())
    for source in args.sources:
        path = Path(source)
        require(not path.is_absolute() and ".." not in path.parts and str(path) == source,
                "Invalid donor source path")
        tree = git("ls-tree", REVISION, "--", source).stdout.decode().strip()
        require(bool(tree), "Source absent from pinned tree: " + source)
        identity, name = tree.split("\t")
        mode, kind, blob = identity.split()
        require(name == source and mode in ("100644", "100755") and kind == "blob", "Invalid source tree entry")
        data, destination, method = read_source(source, blob)
        require(len(data) <= MAX_FILE, "Source exceeds file cap: " + source)
        computed = hashlib.sha1(b"blob " + str(len(data)).encode() + b"\0" + data).hexdigest()
        require(computed == blob, "Pinned source mismatch: " + source)
        total += len(data) - saved.get(source, {}).get("bytes", 0)
        require(total <= MAX_TOTAL, "T3 inventory exceeds bounded total")
        entry = {"source": source, "path": os.path.relpath(destination, ROOT), "bytes": len(data),
                 "sha256": hashlib.sha256(data).hexdigest(), "git_blob_sha1": blob,
                 "retrieval": method, "shared_qeph_original": destination.is_relative_to(QEPH)}
        if source in saved:
            for key in ("source", "path", "bytes", "sha256", "git_blob_sha1", "shared_qeph_original"):
                require(saved[source][key] == entry[key], "Existing T3 source identity changed")
        else:
            if destination.exists():
                require(destination.read_bytes() == data, "Refuse conflicting retained source")
            else:
                destination.parent.mkdir(parents=True, exist_ok=True)
                with destination.open("xb") as output:
                    output.write(data)
            saved[source] = entry
        # Save each verified source so an interrupted serial retrieval resumes
        # without refetching or silently accepting orphan/conflicting bytes.
        manifest["sources"] = [saved[key] for key in sorted(saved)]
        manifest["source_bytes"] = total
        temporary = manifest_path.with_suffix(".json.tmp")
        temporary.write_text(json.dumps(manifest, indent=2) + "\n")
        temporary.replace(manifest_path)
        print(source, len(data), method, flush=True)


if __name__ == "__main__":
    main()
