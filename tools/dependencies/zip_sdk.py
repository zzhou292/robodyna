"""Extract a pinned SDK ZIP locally with bounded expansion and no installer."""

import argparse
import json
from pathlib import Path, PurePosixPath
import shutil
import stat
import zipfile

from tools.dependencies.cuda_math import admit_archive, file_hash


def extract_sdk(manifest, archive, install):
    admit_archive(archive, manifest)
    if install.exists():
        raise ValueError("Select a new SDK directory; preserve prior extraction")
    with zipfile.ZipFile(archive) as source:
        members = source.infolist()
        if sum(member.file_size for member in members) > manifest["max_expanded_bytes"]:
            raise ValueError("SDK expansion exceeds the declared size limit")
        names = set()
        for member in members:
            name = PurePosixPath(member.filename)
            mode = member.external_attr >> 16
            if (name.is_absolute() or ".." in name.parts or "\\" in member.filename or
                    not name.parts or member.filename in names or stat.S_ISLNK(mode)):
                raise ValueError("Unsafe or duplicated SDK ZIP member: " + member.filename)
            names.add(member.filename)
        install.mkdir(parents=True)
        files = {}
        for member in members:
            destination = install / member.filename
            if member.is_dir():
                destination.mkdir(parents=True, exist_ok=True)
                continue
            destination.parent.mkdir(parents=True, exist_ok=True)
            with source.open(member) as stream, destination.open("xb") as output:
                shutil.copyfileobj(stream, output, 1024 * 1024)
            files[member.filename] = {"bytes": destination.stat().st_size, "sha256": file_hash(destination)}
    receipt = {"schema": "robodyna.zip_sdk.v1", "archive": manifest, "files": files,
               "scope": "Authenticated local extraction only; no package scripts or compiler execution"}
    (install / "sdk.json").write_text(json.dumps(receipt, indent=2) + "\n")
    return receipt


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--archive", type=Path, required=True)
    parser.add_argument("--install", type=Path, required=True)
    args = parser.parse_args()
    receipt = extract_sdk(json.loads(args.manifest.read_text()), args.archive, args.install)
    print("Extracted", len(receipt["files"]), "SDK files without executing package code")


if __name__ == "__main__":
    main()
