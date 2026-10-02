"""Package a freshly linked FMU using the retained native XML generator."""

import argparse
import hashlib
import json
from pathlib import Path, PurePosixPath
import re
import shutil
import subprocess
import xml.etree.ElementTree as ET
import zipfile


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def relative_path(name):
    path = PurePosixPath(name)
    if path.is_absolute() or not path.parts or ".." in path.parts or "\\" in name:
        raise ValueError("Invalid FMU member path: " + name)
    return path


def validate_description(path, identifier, version, mode, guid):
    root = ET.parse(path).getroot()
    if root.tag != "fmiModelDescription" or root.get("fmiVersion") != version + ".0":
        raise ValueError("Generated XML has the wrong FMI version/root")
    if root.get("guid" if version == "2" else "instantiationToken") != guid:
        raise ValueError("Generated XML identity differs from the compiled FMU")
    capability = root.find(mode)
    if capability is None or capability.get("modelIdentifier") != identifier:
        raise ValueError("Generated XML does not describe the requested model/mode")
    if root.find("ModelVariables") is None:
        raise ValueError("Generated XML has no model variables")


def package(spec):
    if spec.get("schema") != "robodyna.fmu_build.v1":
        raise ValueError("Unsupported FMU build specification")
    identifier = spec["identifier"]
    if not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", identifier):
        raise ValueError("Invalid model identifier")
    version = spec["version"]
    if version not in ("2", "3") or spec["mode"] not in ("CoSimulation", "ModelExchange"):
        raise ValueError("Unsupported FMI version/mode")
    platform = "linux64" if version == "2" else "x86_64-linux"
    tree = Path(spec["tree"]).resolve()
    tree.mkdir(parents=True, exist_ok=True)
    if any(tree.iterdir()):
        raise ValueError("FMU output tree must be empty")
    library = tree / "binaries" / platform / (identifier + ".so")
    library.parent.mkdir(parents=True)
    original = Path(spec["library"]).resolve()
    shutil.copyfile(original, library)
    if sha256(original) != sha256(library):
        raise ValueError("Packaging changed the freshly built shared library")
    (tree / "resources").mkdir()
    claimed = {library.relative_to(tree).as_posix()}
    runtime_libraries = []
    for row in spec.get("runtime_libraries", []):
        name = relative_path(row["name"])
        if len(name.parts) != 1 or ".so" not in name.name:
            raise ValueError("FMU runtime library must have an explicit DSO basename")
        runtime_libraries.append({"source": row["source"], "destination": "binaries/" + platform + "/" + name.as_posix()})
    for row in spec["resources"] + spec["licenses"] + runtime_libraries:
        name = relative_path(row["destination"]).as_posix()
        if name in claimed or name == "modelDescription.xml":
            raise ValueError("Duplicate/reserved FMU member: " + name)
        claimed.add(name)
        destination = tree / name
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(row["source"], destination)
        if sha256(row["source"]) != sha256(destination):
            raise ValueError("Packaging changed a declared resource/license")

    helper = Path(spec["helper"]).resolve()
    process = subprocess.run([str(helper), str(library.parent), library.name, str(tree)],
                             cwd=tree, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                             text=True, timeout=120, check=False)
    if process.returncode:
        raise RuntimeError("Native FMU description generation failed:\n" + process.stdout[-32768:])
    description = tree / "modelDescription.xml"
    validate_description(description, identifier, version, spec["mode"], spec["guid"])
    shutil.copyfile(description, spec["description"])

    files = []
    with zipfile.ZipFile(spec["archive"], "x", compression=zipfile.ZIP_DEFLATED, compresslevel=6) as output:
        for path in sorted(tree.rglob("*")):
            if path.is_symlink():
                raise ValueError("FMU output may not retain an external symlink")
            if not path.is_file():
                continue
            name = path.relative_to(tree).as_posix()
            info = zipfile.ZipInfo(name, date_time=(1980, 1, 1, 0, 0, 0))
            info.compress_type = zipfile.ZIP_DEFLATED
            info.external_attr = (0o100755 if path == library else 0o100644) << 16
            with path.open("rb") as source, output.open(info, "w") as destination:
                shutil.copyfileobj(source, destination, length=1 << 20)
            files.append({"path": name, "bytes": path.stat().st_size, "sha256": sha256(path)})
    with zipfile.ZipFile(spec["archive"]) as archive:
        if archive.testzip() is not None or sorted(archive.namelist()) != [row["path"] for row in files]:
            raise ValueError("Final FMU ZIP verification failed")
    receipt = {
        "schema": "robodyna.fmu_package.v1", "identifier": identifier,
        "version": version, "mode": spec["mode"], "guid": spec["guid"],
        "helper_sha256": sha256(helper), "library_sha256": sha256(library),
        "description_sha256": sha256(description), "archive_sha256": sha256(spec["archive"]),
        "files": files, "scope": "Native export and packaging; no trajectory/runtime qualification",
    }
    Path(spec["receipt"]).write_text(json.dumps(receipt, indent=2) + "\n")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("spec", type=Path)
    package(json.loads(parser.parse_args().spec.read_text()))
