"""Extract authenticated SDK packages locally, without installing host packages.

Use the workstation guard for extraction. Package-maintainer scripts are not run.
The manifest uses the existing download helper's components/archive/bytes/SHA256
fields. Runtime and development dependencies remain explicit in the SDK provider.
"""

import argparse
import json
import os
from pathlib import Path
import subprocess

from tools.dependencies.cuda_math import admit_archive, file_hash


def extract_packages(manifest, downloads, install):
    if install.exists():
        raise ValueError("Select a new create-only SDK directory: " + str(install))
    for entry in manifest["components"].values():
        admit_archive(downloads / entry["archive"], entry)
    install.mkdir(parents=True)
    for name, entry in manifest["components"].items():
        subprocess.run(["dpkg-deb", "--extract", str(downloads / entry["archive"]), str(install)], check=True)
        print("extracted verified", name, flush=True)
    files = {}
    for path in sorted(install.rglob("*")):
        relative = path.relative_to(install).as_posix()
        if path.is_symlink():
            files[relative] = {"symlink": os.readlink(path)}
        elif path.is_file():
            files[relative] = {"bytes": path.stat().st_size, "sha256": file_hash(path)}
    receipt = {"schema": "robodyna.local_debian_sdk.v1", "packages": manifest["components"], "files": files,
               "scope": "Extracted package files only; no host installation or compilation qualification"}
    (install / "sdk.json").write_text(json.dumps(receipt, indent=2) + "\n")
    return receipt


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", required=True, type=Path)
    parser.add_argument("--downloads", required=True, type=Path)
    parser.add_argument("--install", required=True, type=Path)
    args = parser.parse_args()
    receipt = extract_packages(json.loads(args.manifest.read_text()), args.downloads, args.install)
    print("SDK ready", args.install, len(receipt["files"]), "files", flush=True)


if __name__ == "__main__":
    main()
