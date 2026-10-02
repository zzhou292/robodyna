"""Build an explicit workspace SDK recipe under the caller's resource guard."""

import argparse
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import subprocess
import tarfile

from tools.dependencies.cuda_math import file_hash, strip_members


def read_cache(path):
    result = {}
    for line in Path(path).read_text().splitlines():
        if not line or line.startswith(("//", "#")) or "=" not in line:
            continue
        key, value = line.split("=", 1)
        if ":" in key:
            result[key.split(":", 1)[0]] = value
    return result


def check_cache(path, expected):
    actual = read_cache(path)
    for key, value in expected.items():
        if actual.get(key) != value:
            raise ValueError("SDK CMake cache mismatch: " + key + "=" + repr(actual.get(key)))
    return {key: actual[key] for key in expected}


def tree_identity(root):
    records = []
    for path in sorted(root.rglob("*")):
        name = path.relative_to(root).as_posix()
        if path.is_symlink():
            if not path.exists():
                raise ValueError("SDK tree contains a dangling symlink: " + str(path))
            records.append({"path": name, "symlink": str(path.readlink())})
        elif path.is_file():
            records.append({"path": name, "bytes": path.stat().st_size, "sha256": file_hash(path)})
    return records


def extract_source(archive, identity, destination):
    """Materialize a pinned source tree, including explicitly pinned gitlinks."""
    if archive.stat().st_size != identity["bytes"] or file_hash(archive) != identity["sha256"]:
        raise ValueError("SDK source archive differs from its reviewed pin")
    if destination.exists() and (destination.is_symlink() or not destination.is_dir() or any(destination.iterdir())):
        raise ValueError("Source destination must be new or an empty archive directory")
    destination.mkdir(parents=True, exist_ok=True)
    with tarfile.open(archive, "r:gz") as stream:
        stream.extractall(destination, members=strip_members(stream))


def execute(recipe_path):
    recipe = json.loads(recipe_path.read_text())
    schemas = {"robodyna.occt_sdk_build_proposal.v1", "robodyna.cmake_sdk_build_proposal.v1"}
    if recipe.get("schema") not in schemas:
        raise ValueError("Unsupported SDK recipe; review a new schema explicitly")
    source = Path(recipe["source_directory"])
    build = Path(recipe["build_directory"])
    install = Path(recipe["install_directory"])
    reports = Path(recipe["report_directory"])
    for path in (source, build, install, reports):
        if not path.is_absolute() or path.exists():
            raise ValueError("SDK outputs must be absolute, new directories: " + str(path))
    archive = recipe_path.parent / recipe["source"]["archive"]
    if archive.stat().st_size != recipe["source"]["bytes"] or file_hash(archive) != recipe["source"]["sha256"]:
        raise ValueError("SDK source archive differs from its reviewed pin")
    reports.mkdir(parents=True)
    extract_source(archive, recipe["source"], source)
    for embedded in recipe.get("embedded_sources", []):
        relative = PurePosixPath(embedded["path"])
        if relative.is_absolute() or ".." in relative.parts or not relative.parts:
            raise ValueError("Embedded source path must remain inside the owning source tree")
        extract_source(recipe_path.parent / embedded["source"]["archive"], embedded["source"], source / relative)
    original = tree_identity(source)
    (reports / "source.json").write_text(json.dumps(original, indent=2) + "\n")
    environment = dict(os.environ)
    for name in recipe.get("unset_environment", []):
        environment.pop(name, None)
    environment.update(recipe.get("environment", {}))
    for phase in ("configure", "build", "install"):
        command = recipe[phase]
        if not isinstance(command, list) or not command or any(not isinstance(word, str) for word in command):
            raise ValueError("SDK phase must be an explicit argument vector")
        with (reports / (phase + ".log")).open("x") as output:
            process = subprocess.run(command, stdout=output, stderr=subprocess.STDOUT, check=False, env=environment)
        (reports / (phase + ".json")).write_text(json.dumps({"command": command, "exit_code": process.returncode}, indent=2) + "\n")
        if process.returncode:
            raise RuntimeError("SDK " + phase + " failed; preserve " + str(reports))
        if phase == "configure":
            admitted = check_cache(build / "CMakeCache.txt", recipe["expected_cache"])
            (reports / "cache.json").write_text(json.dumps(admitted, indent=2) + "\n")
    if tree_identity(source) != original:
        raise ValueError("SDK build changed the retained source tree")
    receipt = {
        "schema": "robodyna.occt_built_sdk.v1" if recipe["schema"] == "robodyna.occt_sdk_build_proposal.v1" else "robodyna.cmake_built_sdk.v1",
        "source": recipe["source"],
        "embedded_sources": recipe.get("embedded_sources", []),
        "recipe_sha256": file_hash(recipe_path),
        "cache_sha256": file_hash(build / "CMakeCache.txt"),
        "expected_cache": admitted,
        "installed_files": tree_identity(install),
        "source_unchanged": True,
        "scope": "Source build/install completed; native consumer admission remains separate",
    }
    (install / "sdk.json").write_text(json.dumps(receipt, indent=2) + "\n")
    print(json.dumps({"completed": True, "install": str(install), "files": len(receipt["installed_files"])}))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("recipe", type=Path)
    execute(parser.parse_args().recipe.resolve())
