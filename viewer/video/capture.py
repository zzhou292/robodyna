"""Authenticate captured states before any media process is started."""

import csv
from dataclasses import dataclass
import hashlib
import io
import json
import math
from pathlib import Path
import re
import struct
from types import MappingProxyType


_SCHEMAS = {
    "robo_dyna.physical_replay_capture.v1": ("final_epoch", "final_time_s"),
    "robo_dyna.recovered_sample_capture.v1": ("final_saved_epoch", "final_saved_time_s"),
}
_COLUMNS = ["sample", "epoch", "accepted_time_s", "file", "bytes", "sha256"]
_TEXT_CAP = 2 * 1024 * 1024


@dataclass(frozen=True)
class Frame:
    sample: int
    epoch: int
    accepted_time_s: float
    file: str
    bytes: int
    sha256: str


@dataclass(frozen=True)
class Capture:
    directory: Path
    metadata: object
    frames: tuple
    manifest_sha256: str
    frame_index_sha256: str


def sha256_file(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _read_text(path):
    with path.open("rb") as stream:
        value = stream.read(_TEXT_CAP + 1)
    if len(value) > _TEXT_CAP:
        raise ValueError(f"Capture text exceeds {_TEXT_CAP} bytes: {path}")
    return value


def _unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"Duplicate JSON key: {key}")
        result[key] = value
    return result


def _positive_integer(value, name):
    if type(value) is not int or value <= 0:
        raise ValueError(f"Invalid positive integer {name}")
    return value


def _finite_time(value):
    if isinstance(value, bool) or not isinstance(value, (float, int)):
        raise ValueError("Invalid physical time")
    if not math.isfinite(value) or value < 0:
        raise ValueError("Physical time must be finite and nonnegative")
    return value


def _freeze(value):
    if isinstance(value, dict):
        return MappingProxyType({key: _freeze(item) for key, item in value.items()})
    if isinstance(value, list):
        return tuple(_freeze(item) for item in value)
    return value


def metadata_dict(capture):
    """Return a JSON-ready copy; the validated capture itself stays immutable."""
    def thaw(value):
        if isinstance(value, MappingProxyType):
            return {key: thaw(item) for key, item in value.items()}
        if isinstance(value, tuple):
            return [thaw(item) for item in value]
        return value
    return thaw(capture.metadata)


def validate_capture(capture_dir):
    """Return a verified immutable Capture, or raise before creating output.

    Capture completion is required. Simulation horizon completion is independent
    and is retained exactly, including unavailable recovered interval history.
    """
    directory = Path(capture_dir).resolve(strict=True)
    manifest_bytes = _read_text(directory / "manifest.json")
    index_bytes = _read_text(directory / "frames.csv")
    metadata = json.loads(manifest_bytes, object_pairs_hook=_unique_object)
    if not isinstance(metadata, dict) or metadata.get("schema") not in _SCHEMAS:
        raise ValueError("Unsupported Chrono capture schema")
    if (metadata.get("complete_capture") is not True
            or metadata.get("interpolated_frames") is not False
            or metadata.get("simulation_executed_by_viewer") is not False
            or type(metadata.get("deformation_scale")) not in (int, float)
            or metadata["deformation_scale"] != 1):
        raise ValueError("Capture must be complete, physical scale 1 and noninterpolated")
    if metadata.get("all_png_decoded") is not True:
        raise ValueError("Capture lacks completed PNG decode verification")
    if metadata["schema"] == "robo_dyna.recovered_sample_capture.v1":
        if (metadata.get("interval_ledger_available") is not False
                or metadata.get("continuous_accepted_history_available") is not False
                or metadata.get("input_horizon_completion") != "unknown"):
            raise ValueError("Recovered capture must retain unavailable history and unknown horizon")
    count = _positive_integer(metadata.get("frames"), "frames")
    width = _positive_integer(metadata.get("width"), "width")
    height = _positive_integer(metadata.get("height"), "height")
    if width % 2 or height % 2:
        raise ValueError("H264 yuv420p requires even captured dimensions")
    index_hash = hashlib.sha256(index_bytes).hexdigest()
    if index_hash != metadata.get("frame_index_sha256"):
        raise ValueError("frames.csv digest mismatch")
    reader = csv.DictReader(io.StringIO(index_bytes.decode("utf-8"), newline=""))
    if reader.fieldnames != _COLUMNS:
        raise ValueError("Unexpected frames.csv columns")
    frames = []
    total_bytes = 0
    for number, row in enumerate(reader):
        if None in row or any(row[key] is None for key in _COLUMNS):
            raise ValueError("Malformed frame row")
        if any(re.fullmatch(r"[0-9]+", row[key]) is None
               for key in ("sample", "epoch", "bytes")):
            raise ValueError("Malformed frame integer")
        frame = Frame(int(row["sample"]), int(row["epoch"]),
                      float(row["accepted_time_s"]), row["file"],
                      int(row["bytes"]), row["sha256"])
        _finite_time(frame.accepted_time_s)
        if frame.sample != number or frame.file != f"frame-{number:06d}.png":
            raise ValueError("Frame order or filename mismatch")
        if frames and (frame.epoch <= frames[-1].epoch
                       or frame.accepted_time_s <= frames[-1].accepted_time_s):
            raise ValueError("Frame epochs and physical times must increase strictly")
        if frame.bytes <= 0 or re.fullmatch(r"[0-9a-f]{64}", frame.sha256) is None:
            raise ValueError("Invalid PNG size or digest")
        image = directory / frame.file
        if image.stat().st_size != frame.bytes or sha256_file(image) != frame.sha256:
            raise ValueError(f"PNG size or digest mismatch: {frame.file}")
        with image.open("rb") as stream:
            header = stream.read(24)
        if (len(header) != 24 or header[:16] != b"\x89PNG\r\n\x1a\n\x00\x00\x00\rIHDR"
                or struct.unpack(">II", header[16:24]) != (width, height)):
            raise ValueError(f"PNG dimensions or header mismatch: {frame.file}")
        frames.append(frame)
        total_bytes += frame.bytes
    if len(frames) != count or total_bytes != metadata.get("png_bytes"):
        raise ValueError("Capture frame count or total PNG bytes mismatch")
    epoch_key, time_key = _SCHEMAS[metadata["schema"]]
    final_time = _finite_time(metadata.get(time_key))
    if (type(metadata.get(epoch_key)) is not int
            or frames[-1].epoch != metadata[epoch_key]
            or frames[-1].accepted_time_s != final_time):
        raise ValueError("Final captured epoch or physical time mismatch")
    return Capture(directory, _freeze(metadata), tuple(frames),
                   hashlib.sha256(manifest_bytes).hexdigest(), index_hash)
