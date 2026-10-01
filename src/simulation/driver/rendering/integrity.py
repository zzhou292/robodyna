"""Bounded inventory of an immutable accepted directory; used inside the guard."""

from pathlib import Path
from viewer.file_integrity import sha256_file
from ..jsonio import require

MAX_FILES = 10000
MAX_BYTES = 7 << 30  # Native 6 GiB archive plus bounded controller sidecars.


def inventory(directory):
    root = Path(directory).resolve(strict=True)
    require(root.is_dir(), "accepted input directory is missing")
    records, total = [], 0
    for path in root.rglob("*"):
        require(not path.is_symlink(), "immutable accepted input contains a symbolic link")
        if path.is_dir():
            continue
        require(path.is_file(), "immutable accepted input contains a nonregular file")
        require(len(records) < MAX_FILES, "accepted directory exceeds inventory file limit")
        size = path.stat().st_size
        require(size <= MAX_BYTES - total, "accepted directory exceeds native archive inventory budget")
        total += size
        records.append(dict(file=str(path.relative_to(root)), bytes=size, sha256=sha256_file(path)))
    records.sort(key=lambda value: value["file"])
    return dict(schema="robodyna.accepted_directory_inventory.v1", root=str(root),
                files=records, file_count=len(records), total_bytes=total)
