"""Authenticate only the visualization assets consumed by the native viewer."""

from pathlib import Path, PurePosixPath
import re

from viewer.file_integrity import sha256_file
from ..jsonio import fields, integer, require, resolve
from ..runtime import runtime_file

ANCHOR = "logo_chrono_alpha.png"
FONT = "vsg/fonts/OpenSans-Bold.vsgb"
MAX_FILES = 4096
MAX_BYTES = 256 << 20


def _name(text):
    require(type(text) is str and 0 < len(text) <= 4096 and "\\" not in text and "\x00" not in text,
            "invalid visualization asset name")
    path = PurePosixPath(text)
    require(not path.is_absolute() and ".." not in path.parts and str(path) == text
            and (text == ANCHOR or text.startswith(("vsg/", "colormaps/"))),
            "asset inventory must stay inside the declared visualization roots")
    return text


def _actual_files(root):
    files = {ANCHOR}
    for name in ("vsg", "colormaps"):
        directory = root / name
        if not directory.exists():
            continue
        require(directory.is_dir() and not directory.is_symlink(), "visualization asset root must be a real directory")
        for path in directory.rglob("*"):
            require(not path.is_symlink(), "visualization asset inventory cannot follow symbolic links")
            if path.is_dir():
                continue
            require(path.is_file(), "visualization asset must be a regular file")
            files.add(str(path.relative_to(root)))
            require(len(files) <= MAX_FILES, "visualization asset inventory exceeds file limit")
    return files


def load_assets(value, base):
    fields(value, ("anchor", "files"))
    anchor = value["anchor"]
    require(type(anchor) is dict and set(anchor) in ({"path"}, {"runfile"}),
            "visualization data needs one explicit file anchor")
    if "runfile" in anchor:
        require(type(anchor["runfile"]) is str and 0 < len(anchor["runfile"]) <= 4096, "invalid asset runfile")
        path = runtime_file(anchor["runfile"])
    else:
        path = resolve(base, anchor["path"])
    require(path.name == ANCHOR and path.is_file(), "visualization anchor must be logo_chrono_alpha.png")
    # Resolve the file anchor rather than requiring a directory entry in the
    # runfiles manifest. All declared files must form this actual data tree.
    root = path.resolve(strict=True).parent
    require(type(value["files"]) is list and 2 <= len(value["files"]) <= MAX_FILES, "invalid asset inventory size")
    records, total = {}, 0
    for entry in value["files"]:
        fields(entry, ("file", "bytes", "sha256"))
        name = _name(entry["file"])
        require(name not in records, "duplicate visualization asset")
        size = integer(entry["bytes"], "visualization asset bytes", 1, MAX_BYTES)
        total += size
        require(total <= MAX_BYTES, "visualization assets exceed 256 MiB admission")
        digest = entry["sha256"]
        require(type(digest) is str and re.fullmatch(r"[0-9a-f]{64}", digest), "invalid asset SHA-256")
        target = root / name
        require(target.is_file() and not target.is_symlink() and target.resolve().is_relative_to(root)
                and target.stat().st_size == size and sha256_file(target) == digest,
                f"visualization asset identity differs: {name}")
        records[name] = dict(path=str(target), bytes=size, sha256=digest)
    require(ANCHOR in records and FONT in records, "native viewer requires authenticated logo and font")
    require(set(records) == _actual_files(root), "visualization asset inventory is incomplete or unexpected")
    return str(root), records
