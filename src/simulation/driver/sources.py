"""Stream authenticated source files and ZIP members without running a solver."""

import hashlib
from pathlib import Path, PurePosixPath
import stat
import zipfile

from viewer.file_integrity import sha256_file
from .jsonio import require


def member_basename(name):
    require(type(name) is str and name and "\x00" not in name and "\\" not in name,
            "invalid original ZIP member name")
    path = PurePosixPath(name)
    require(not path.is_absolute() and ".." not in path.parts and not name.endswith("/")
            and path.name not in ("", ".", ".."), "unsafe original ZIP member name")
    return path.name


def verify_file(pin):
    path = Path(pin["path"])
    require(not path.is_symlink() and path.is_file(), f"regular source file required: {path}")
    require(path.stat().st_size == pin["bytes"] and sha256_file(path) == pin["sha256"],
            f"source file identity changed: {path}")


def _member(archive, spec, output=None):
    matches = [info for info in archive.infolist() if info.filename == spec["member"]]
    require(len(matches) == 1 and not matches[0].is_dir(), "missing or duplicate original ZIP member")
    require(not stat.S_ISLNK(matches[0].external_attr >> 16), "original ZIP member must not be a symbolic link")
    require(matches[0].file_size == spec["bytes"], "original member extent differs")
    digest, count = hashlib.sha256(), 0
    with archive.open(matches[0]) as stream:
        while block := stream.read(1024 * 1024):
            count += len(block)
            require(count <= spec["bytes"], "original member exceeds its byte cap")
            digest.update(block)
            if output is not None:
                output.write(block)
    require(count == spec["bytes"] and digest.hexdigest() == spec["sha256"], "original member identity differs")


def verify_sources(case):
    require(case.canonical_directory.is_dir() and not case.canonical_directory.is_symlink(),
            "real canonical source directory required")
    for value in case.files.values():
        verify_file(value)
    with zipfile.ZipFile(case.files["source_archive"]["path"]) as archive:
        for value in case.members.values():
            _member(archive, value)
    return dict(schema="robodyna.validation.v1", profile="yaris.native_v6.wall_self",
                manifest=str(case.manifest), checked_file_pins=len(case.files), checked_original_members=len(case.members),
                scope="manifest_and_named_source_identity_only", physics_prepared=False,
                canonical_arrays_authenticated=False, gpu_used=False)


def extract_members(case, destination):
    destination = Path(destination)
    names = {role: member_basename(spec["member"]) for role, spec in case.members.items()}
    require(len(set(names.values())) == len(names), "duplicate original source basenames")
    destination.mkdir()
    paths = {}
    with zipfile.ZipFile(case.files["source_archive"]["path"]) as archive:
        for role, spec in case.members.items():
            # SourceInputs retains this basename as provenance. Directory
            # components are never used to choose a filesystem destination.
            target = destination / names[role]
            with target.open("xb") as stream:
                _member(archive, spec, stream)
            paths[role] = str(target)
    return paths
