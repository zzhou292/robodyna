"""Bounded artifact admission shared by the parity readers."""
from dataclasses import dataclass
from pathlib import Path
import math
import re

from benchmarks.accepted_payloads.raw_json import read_object
from viewer.file_integrity import sha256_file

JSON_CAP = 1 << 20
MAX_FILES = 512
MAX_TOTAL_BYTES = 8 << 30
SHA = re.compile(r"[0-9a-f]{64}\Z")


def require(condition, message):
    if not condition:
        raise ValueError(message)


def keys(value, expected, label):
    require(isinstance(value, dict) and set(value) == set(expected),
            f"{label}: missing or unknown fields")


def text(value, label):
    require(isinstance(value, str) and 0 < len(value) <= 512, f"{label}: text required")
    return value


def finite_tree(value):
    if isinstance(value, float):
        require(math.isfinite(value), "nonfinite JSON number")
    elif isinstance(value, dict):
        for child in value.values():
            finite_tree(child)
    elif isinstance(value, list):
        for child in value:
            finite_tree(child)


@dataclass(frozen=True)
class Artifact:
    path: Path
    sha256: str
    bytes: int


class Artifacts:
    """A bounded, read-only comparison invocation, including retained code pins."""
    def __init__(self):
        self._known = {}
        self._bytes = 0

    def verify(self, value, parent):
        require(isinstance(value, dict) and set(value) in (
            {"path", "sha256"}, {"path", "bytes", "sha256"}), "invalid artifact pin fields")
        text(value["path"], "artifact path")
        require(isinstance(value["sha256"], str) and SHA.fullmatch(value["sha256"]) is not None, "invalid SHA256")
        path = Path(value["path"])
        path = path if path.is_absolute() else Path(parent) / path
        require(not path.is_symlink() and path.is_file(), f"regular nonsymlink file required: {path}")
        path = path.resolve()
        identity = (path, value["sha256"])
        size = path.stat().st_size
        require(size > 0, f"empty evidence file: {path}")
        if "bytes" in value:
            require(type(value["bytes"]) is int and value["bytes"] == size,
                    f"pinned artifact extent differs: {path}")
        if identity not in self._known:
            require(len(self._known) < MAX_FILES and size <= MAX_TOTAL_BYTES - self._bytes,
                    "comparison artifact budget exceeded")
            require(sha256_file(path) == value["sha256"], f"stale artifact hash: {path}")
            self._known[identity] = Artifact(path, value["sha256"], size)
            self._bytes += size
        else:
            require(size == self._known[identity].bytes, f"artifact extent changed: {path}")
        return self._known[identity]

    def object(self, artifact, cap=JSON_CAP):
        value, _ = read_object(artifact.path, max_bytes=cap)
        finite_tree(value)
        require(sha256_file(artifact.path) == artifact.sha256, f"artifact changed during read: {artifact.path}")
        return value

    def recheck(self):
        for artifact in self._known.values():
            require(artifact.path.stat().st_size == artifact.bytes and
                    sha256_file(artifact.path) == artifact.sha256,
                    f"artifact changed during comparison: {artifact.path}")

    def inventory(self):
        return [{"path": str(v.path), "bytes": v.bytes, "sha256": v.sha256}
                for v in sorted(self._known.values(), key=lambda row: str(row.path))]
