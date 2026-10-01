"""Create an authenticated presentation derivative of an existing movie."""

import argparse
from fractions import Fraction
import json
from pathlib import Path
import struct
import subprocess
from types import SimpleNamespace

from ..file_integrity import sha256_file
from .capture import _read_text, _unique_object
from .encode import _tools, _verify_probe, _write_json


def _record(path):
    return {"path": str(path), "bytes": path.stat().st_size, "sha256": sha256_file(path)}


def _source(directory):
    directory = Path(directory).resolve(strict=True)
    manifest = directory / "manifest.json"
    value = json.loads(_read_text(manifest), object_pairs_hook=_unique_object)
    if (value.get("schema") != "robo_dyna.chrono_capture_video.v1"
            or value.get("full_decode_passed") is not True):
        raise ValueError("Branding requires a completed authenticated capture movie")
    video = value.get("video", {})
    if video.get("file") != "movie.mp4":
        raise ValueError("Source movie must use the qualified local movie.mp4 filename")
    movie = directory / "movie.mp4"
    if movie.is_symlink() or not movie.is_file():
        raise ValueError("Source movie must be a regular file")
    for key in ("frames", "fps", "bytes"):
        if type(video.get(key)) is not int or video[key] <= 0:
            raise ValueError(f"Invalid source movie {key}")
    metadata = value.get("capture_metadata", {})
    width, height = metadata.get("width"), metadata.get("height")
    if (type(width) is not int or type(height) is not int
            or not 248 <= width <= 4096 or not 114 <= height <= 4096
            or width % 2 or height % 2):
        raise ValueError("Source dimensions cannot contain the fixed branding panel")
    record = _record(movie)
    if record["bytes"] != video["bytes"] or record["sha256"] != video.get("sha256"):
        raise ValueError("Source movie differs from its completed manifest")
    return value, movie, record, _record(manifest)


def _logo(path):
    path = Path(path).resolve(strict=True)
    if not path.is_file() or path.stat().st_size > 32 * 1024 * 1024:
        raise ValueError("Logo must be a bounded PNG file")
    with path.open("rb") as stream:
        header = stream.read(33)
    if len(header) != 33 or header[:16] != b"\x89PNG\r\n\x1a\n\x00\x00\x00\rIHDR":
        raise ValueError("Logo must have a valid PNG header")
    width, height = struct.unpack(">II", header[16:24])
    if not 0 < width <= 4096 or not 0 < height <= 4096:
        raise ValueError("Logo dimensions exceed the bounded media profile")
    return path, _record(path)


def _run(command, log, output=None):
    with log.open("xb") as errors:
        if output is None:
            subprocess.run(command, check=True, stdout=subprocess.DEVNULL, stderr=errors)
        else:
            with output.open("xb") as stream:
                subprocess.run(command, check=True, stdout=stream, stderr=errors)


def _probe(tool, movie, destination, name, source):
    command = [str(tool), "-v", "error", "-threads", "1", "-count_frames",
               "-show_streams", "-show_format", "-of", "json", str(movie)]
    path = destination / f"{name}.json"
    _run(command, destination / f"{name}.log", path)
    probe = json.loads(_read_text(path), object_pairs_hook=_unique_object)
    _verify_probe(probe, SimpleNamespace(metadata=source["capture_metadata"]),
                  source["video"]["frames"], source["video"]["fps"])
    return probe, command


def brand_video(source_dir, output_dir, logo, ffmpeg, poster_frame):
    """Preserve movie cadence and canvas; overlay a declared top-right panel.

    Output is create-only. Originals, captures and qualification receipts are
    never modified. Failed work retains diagnostics without a success manifest.
    """
    source, original, original_record, source_record = _source(source_dir)
    count, fps = source["video"]["frames"], source["video"]["fps"]
    if type(poster_frame) is not int or not 0 <= poster_frame < count:
        raise ValueError("Poster frame must select an existing zero-based movie frame")
    logo, logo_record = _logo(logo)
    encoder, probe_tool = _tools(ffmpeg)
    tools = [_record(encoder), _record(probe_tool)]
    destination = Path(output_dir).absolute()
    destination.mkdir()
    original_probe, source_probe_command = _probe(
        probe_tool, original, destination, "source-ffprobe", source)
    movie = destination / "movie.mp4"
    # The opaque panel covers the old UI logo. Only the logo input is rescaled;
    # the simulation image is neither cropped, rescaled nor interpolated.
    filter_graph = ("[0:v]drawbox=x=iw-248:y=8:w=240:h=106:color=white:t=fill[panel];"
                    "[1:v]scale=224:90:flags=lanczos,setsar=1[logo];"
                    "[panel][logo]overlay=x=main_w-240:y=16:eof_action=repeat:shortest=0[out]")
    command = [str(encoder), "-nostdin", "-n", "-threads", "1",
               "-filter_threads", "1", "-filter_complex_threads", "1",
               "-i", str(original), "-threads", "1", "-i", str(logo),
               "-filter_complex", filter_graph, "-map", "[out]", "-an",
               "-vsync", "0", "-c:v", "libx264", "-threads", "1",
               "-preset", "medium", "-crf", "22", "-pix_fmt", "yuv420p",
               "-movflags", "+faststart", str(movie)]
    _run(command, destination / "encode.log")
    result_probe, probe_command = _probe(probe_tool, movie, destination, "ffprobe", source)
    original_stream, result_stream = original_probe["streams"][0], result_probe["streams"][0]
    if (Fraction(original_stream["time_base"]) * original_stream["duration_ts"]
            != Fraction(result_stream["time_base"]) * result_stream["duration_ts"]):
        raise ValueError("Branding changed exact movie track duration")
    decode_command = [str(encoder), "-nostdin", "-v", "error", "-xerror", "-threads", "1",
                      "-i", str(movie), "-threads", "1", "-f", "null", "-"]
    _run(decode_command, destination / "decode.log")
    poster = destination / "poster.png"
    poster_command = [str(encoder), "-nostdin", "-n", "-v", "error", "-xerror",
                      "-threads", "1", "-filter_threads", "1", "-i", str(movie),
                      "-vf", f"select=eq(n\\,{poster_frame})", "-vsync", "0",
                      "-frames:v", "1", "-threads", "1", str(poster)]
    _run(poster_command, destination / "poster.log")
    poster_decode_command = [str(encoder), "-nostdin", "-v", "error", "-xerror",
                             "-threads", "1", "-i", str(poster), "-threads", "1",
                             "-f", "null", "-"]
    _run(poster_decode_command, destination / "poster-decode.log")
    # Late checks bind the derivative to the actual stable inputs and tools.
    for record in [original_record, source_record, logo_record, *tools]:
        if _record(Path(record["path"])) != record:
            raise ValueError("Source, logo or media tools changed during branding")
    receipt = {
        "schema": "robodyna.branded_video.v1", "complete": True,
        "source_movie": original_record, "source_manifest": source_record,
        "source_capture_metadata": source["capture_metadata"], "logo": logo_record,
        "tools": tools, "video": {**_record(movie), "file": movie.name,
                                    "frames": count, "fps": fps,
                                    "duration_s": count / fps},
        "poster": {**_record(poster), "file": poster.name, "video_frame": poster_frame},
        "branding": {"panel": {"x": source["capture_metadata"]["width"] - 248,
                                "y": 8, "width": 240, "height": 106, "color": "white"},
                     "logo": {"x": source["capture_metadata"]["width"] - 240,
                              "y": 16, "width": 224, "height": 90}},
        "simulation_executed": False, "frame_cadence_changed": False,
        "interpolated_frames": False, "full_decode_passed": True,
        "poster_decode_passed": True,
        "presentation": "Branding derivative only; original simulation evidence is retained unchanged.",
        "encoding": "Lossy H264/yuv420p re-encode; opaque top-right branding panel; no scene crop or rescale.",
        "encode_command": command, "source_probe_command": source_probe_command,
        "probe_command": probe_command, "decode_command": decode_command,
        "poster_command": poster_command, "poster_decode_command": poster_decode_command,
    }
    _write_json(destination / "manifest.json", receipt)
    return movie


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source")
    parser.add_argument("output")
    parser.add_argument("--logo", required=True)
    parser.add_argument("--ffmpeg", required=True)
    parser.add_argument("--poster-frame", required=True, type=int)
    args = parser.parse_args(argv)
    print(brand_video(args.source, args.output, args.logo, args.ffmpeg, args.poster_frame))


if __name__ == "__main__":
    main()
