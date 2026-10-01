"""Bind a native movie to its existing captured states and exact hold plan."""

from dataclasses import asdict, dataclass
import json
from pathlib import Path

from .brand import _record
from .capture import _read_text, _unique_object, metadata_dict, validate_capture
from .encode import hold_boundaries


@dataclass(frozen=True)
class NativeMovie:
    manifest: dict
    movie: Path
    movie_record: dict
    manifest_record: dict
    capture_directory: Path


def read_native_movie(directory):
    """Authenticate the native encoder's source closure before creating output.

    Publication does not infer physical horizon completion from capture completion.
    The original capture metadata, including incomplete/unknown horizons, survives.
    """
    directory = Path(directory).resolve(strict=True)
    manifest_path = directory / "manifest.json"
    if manifest_path.is_symlink() or not manifest_path.is_file():
        raise ValueError("Native movie manifest must be a regular file")
    value = json.loads(_read_text(manifest_path), object_pairs_hook=_unique_object)
    if (not isinstance(value, dict)
            or value.get("schema") != "robo_dyna.chrono_capture_video.v1"
            or value.get("full_decode_passed") is not True
            or value.get("interpolated_frames") is not False
            or type(value.get("deformation_scale")) not in (int, float)
            or value["deformation_scale"] != 1):
        raise ValueError("Publication requires a verified native capture movie at scale 1")
    video = value.get("video", {})
    if not isinstance(video, dict) or video.get("file") != "movie.mp4":
        raise ValueError("Native movie must use its local movie.mp4 filename")
    for key in ("frames", "fps", "bytes"):
        if type(video.get(key)) is not int or video[key] <= 0:
            raise ValueError(f"Invalid native movie {key}")
    movie = directory / "movie.mp4"
    if movie.is_symlink() or not movie.is_file():
        raise ValueError("Native movie must be a regular file")
    movie_record = _record(movie)
    if (movie_record["bytes"] != video["bytes"]
            or movie_record["sha256"] != video.get("sha256")):
        raise ValueError("Native movie differs from its completed encoder manifest")
    capture_name = value.get("capture")
    if not isinstance(capture_name, str) or not capture_name or "\x00" in capture_name:
        raise ValueError("Native movie requires its retained capture directory")
    capture_path = Path(capture_name)
    if not capture_path.is_absolute():
        capture_path = directory / capture_path
    capture = validate_capture(capture_path)
    if (metadata_dict(capture) != value.get("capture_metadata")
            or capture.manifest_sha256 != value.get("capture_manifest_sha256")
            or capture.frame_index_sha256 != value.get("frame_index_sha256")):
        raise ValueError("Native movie capture provenance differs from retained source")
    boundaries = hold_boundaries(len(capture.frames), value.get("samples_per_second"), video["fps"])
    holds = [{**asdict(frame), "first_video_frame": begin,
              "video_frames": end - begin, "hold_duration_s": (end - begin) / video["fps"]}
             for frame, begin, end in zip(capture.frames, boundaries, boundaries[1:])]
    if (value.get("recorded_samples") != holds or video["frames"] != boundaries[-1]
            or type(video.get("duration_s")) not in (int, float)
            or video["duration_s"] != video["frames"] / video["fps"]):
        raise ValueError("Native movie differs from its exact captured-state hold plan")
    if any(not 0 < capture.metadata[key] <= 4096 for key in ("width", "height")):
        raise ValueError("Native movie exceeds the bounded publication dimensions")
    return NativeMovie(value, movie, movie_record, _record(manifest_path), capture.directory)
