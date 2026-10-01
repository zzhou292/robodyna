"""Create an overlay-free, size-admitted delivery copy of a native movie."""

import argparse
from pathlib import Path
import shutil

# These existing media primitives perform hashing, bounded probing and checked
# subprocess calls. The opaque brand_video transformation is never invoked.
from .brand import _probe, _record, _run
from .encode import _tools, _write_json
from .publication_source import read_native_movie


MAX_PUBLICATION_BYTES = 10_000_000


def export_video(source_dir, output_dir, ffmpeg, poster_frame, max_bytes=MAX_PUBLICATION_BYTES):
    """Copy small movies exactly, or compress once at CRF22 without any overlay.

    The output is create-only. If CRF22 still exceeds the declared size limit,
    retain diagnostics and reject; never silently lower quality or trim frames.
    """
    if type(max_bytes) is not int or not 0 < max_bytes <= MAX_PUBLICATION_BYTES:
        raise ValueError("Publication byte limit must be positive and at most 10000000")
    source = read_native_movie(source_dir)
    native = source.manifest
    count, fps = native["video"]["frames"], native["video"]["fps"]
    if type(poster_frame) is not int or not 0 <= poster_frame < count:
        raise ValueError("Poster frame must select an existing zero-based movie frame")
    encoder, probe_tool = _tools(ffmpeg)
    tools = [_record(encoder), _record(probe_tool)]
    destination = Path(output_dir).absolute()
    if (destination.resolve().is_relative_to(source.movie.parent)
            or destination.resolve().is_relative_to(source.capture_directory)):
        raise ValueError("Publication output must be outside its preserved inputs")
    destination.mkdir()
    _, source_probe_command = _probe(probe_tool, source.movie, destination, "source-ffprobe", native)
    movie = destination / "movie.mp4"
    compressed = source.movie_record["bytes"] > max_bytes
    encode_command = None
    if compressed:
        encode_command = [str(encoder), "-nostdin", "-n", "-v", "error", "-xerror",
                          "-threads", "1", "-filter_threads", "1", "-i", str(source.movie),
                          "-map", "0:v:0", "-an", "-vsync", "0", "-c:v", "libx264",
                          "-threads", "1", "-preset", "medium", "-crf", "22",
                          "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(movie)]
        _run(encode_command, destination / "encode.log")
    else:
        with source.movie.open("rb") as input_stream, movie.open("xb") as output_stream:
            shutil.copyfileobj(input_stream, output_stream, length=1 << 20)
    movie_record = _record(movie)
    if not 0 < movie_record["bytes"] <= max_bytes:
        raise ValueError("Publication movie exceeds the byte limit after CRF22; no delivery manifest published")
    if not compressed and movie_record["sha256"] != source.movie_record["sha256"]:
        raise ValueError("Exact publication copy differs from its source")
    _, probe_command = _probe(probe_tool, movie, destination, "ffprobe", native)
    decode_command = [str(encoder), "-nostdin", "-v", "error", "-xerror", "-threads", "1",
                      "-i", str(movie), "-threads", "1", "-f", "null", "-"]
    _run(decode_command, destination / "decode.log")
    poster = destination / "poster.png"
    poster_command = [str(encoder), "-nostdin", "-n", "-v", "error", "-xerror",
                      "-threads", "1", "-filter_threads", "1", "-i", str(movie),
                      "-vf", f"select=eq(n\\,{poster_frame})", "-vsync", "0",
                      "-frames:v", "1", "-threads", "1", str(poster)]
    _run(poster_command, destination / "poster.log")
    poster_record = _record(poster)
    poster_decode_command = [str(encoder), "-nostdin", "-v", "error", "-xerror", "-threads", "1",
                             "-i", str(poster), "-threads", "1", "-f", "null", "-"]
    _run(poster_decode_command, destination / "poster-decode.log")
    for record in [source.movie_record, source.manifest_record, *tools, movie_record, poster_record]:
        if _record(Path(record["path"])) != record:
            raise ValueError("Publication input, output or tool changed during export")
    # Reuses the capture validator, including every PNG hash, without pretending
    # that a completed visualization capture certifies the numerical solution.
    if read_native_movie(source_dir) != source:
        raise ValueError("Native capture provenance changed during export")
    receipt = {
        "schema": "robodyna.publication_video.v1", "complete": True,
        "source_movie": source.movie_record, "source_manifest": source.manifest_record,
        "source_capture_metadata": native["capture_metadata"],
        "source_capture_manifest_sha256": native["capture_manifest_sha256"],
        "source_frame_index_sha256": native["frame_index_sha256"],
        "tools": tools, "maximum_video_bytes": max_bytes,
        "video": {**movie_record, "file": movie.name, "frames": count, "fps": fps,
                  "duration_s": count / fps},
        "poster": {**poster_record, "file": poster.name, "video_frame": poster_frame},
        "encoding": {"operation": "crf22_reencode" if compressed else "byte_copy",
                     "lossy_reencode": compressed, "crf": 22 if compressed else None},
        "overlay_applied": False, "simulation_executed": False,
        "frame_cadence_changed": False, "interpolated_frames": False,
        "full_decode_passed": True, "poster_decode_passed": True,
        "presentation": "Delivery copy of the native render; no overlay, scene crop, rescale or geometric interpolation. Existing capture/physics claims are retained as provenance, not newly qualified.",
        "source_probe_command": source_probe_command, "encode_command": encode_command,
        "probe_command": probe_command, "decode_command": decode_command,
        "poster_command": poster_command, "poster_decode_command": poster_decode_command,
    }
    _write_json(destination / "manifest.json", receipt)
    return movie


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source")
    parser.add_argument("output")
    parser.add_argument("--ffmpeg", required=True)
    parser.add_argument("--poster-frame", required=True, type=int)
    parser.add_argument("--max-bytes", type=int, default=MAX_PUBLICATION_BYTES)
    args = parser.parse_args(argv)
    print(export_video(args.source, args.output, args.ffmpeg, args.poster_frame, args.max_bytes))


if __name__ == "__main__":
    main()
