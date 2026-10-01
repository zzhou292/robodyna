"""Encode exact saved states with explicit holds, then verify the movie."""

import argparse
from dataclasses import asdict
from fractions import Fraction
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess

from ..file_integrity import sha256_file
from .capture import metadata_dict, validate_capture


def hold_boundaries(sample_count, samples_per_second, output_fps):
    """Cumulative ceil(i * fps / sample_rate), with at least one frame/state."""
    if type(output_fps) is not int or output_fps <= 0:
        raise ValueError("output_fps must be a positive integer")
    if (isinstance(samples_per_second, bool)
            or not isinstance(samples_per_second, (float, int))
            or not math.isfinite(samples_per_second)
            or not 0 < samples_per_second <= output_fps):
        raise ValueError("samples_per_second must be finite, positive and <= output_fps")
    rate = Fraction(str(samples_per_second))
    step = output_fps / rate
    return tuple((i * step.numerator + step.denominator - 1) // step.denominator
                 for i in range(sample_count + 1))


def _tools(ffmpeg):
    encoder = Path(ffmpeg).resolve(strict=True)
    probe = encoder.with_name("ffprobe").resolve(strict=True)
    for path in (encoder, probe):
        if not path.is_file() or not os.access(path, os.X_OK):
            raise ValueError(f"Media tool is not executable: {path}")
    return encoder, probe


def _write_json(path, value):
    with path.open("x", encoding="utf-8") as stream:
        json.dump(value, stream, indent=2, allow_nan=False)
        stream.write("\n")


def _stream_frames(command, capture, boundaries, log):
    # Repetition happens before the decoder. No fps filter, blending or image
    # resampling participates in choosing which captured state is shown.
    with log.open("xb") as errors:
        process = subprocess.Popen(command, stdin=subprocess.PIPE,
                                   stdout=subprocess.DEVNULL, stderr=errors)
        try:
            for frame, begin, end in zip(capture.frames, boundaries, boundaries[1:]):
                for _ in range(end - begin):
                    digest = hashlib.sha256()
                    size = 0
                    with (capture.directory / frame.file).open("rb") as image:
                        for chunk in iter(lambda: image.read(1024 * 1024), b""):
                            process.stdin.write(chunk)
                            digest.update(chunk)
                            size += len(chunk)
                    if size != frame.bytes or digest.hexdigest() != frame.sha256:
                        raise ValueError(f"Captured PNG changed during encoding: {frame.file}")
            process.stdin.close()
            return_code = process.wait()
            if return_code:
                raise subprocess.CalledProcessError(return_code, command)
        except BaseException:
            if process.poll() is None:
                process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
            process.stdin.close()
            raise


def _verify_probe(probe, capture, count, fps):
    streams = probe.get("streams", [])
    if len(streams) != 1:
        raise ValueError("Movie must contain exactly one video stream")
    stream = streams[0]
    if (stream.get("codec_type") != "video" or stream.get("codec_name") != "h264"
            or stream.get("pix_fmt") != "yuv420p"
            or (stream.get("width"), stream.get("height"))
            != (capture.metadata["width"], capture.metadata["height"])
            or int(stream.get("nb_read_frames", -1)) != count
            or Fraction(stream.get("avg_frame_rate", "0")) != fps
            or Fraction(stream.get("r_frame_rate", "0")) != fps):
        raise ValueError("Encoded stream does not match the captured-state hold plan")
    ticks = stream.get("duration_ts")
    start = stream.get("start_pts")
    if type(ticks) is not int or ticks <= 0 or type(start) is not int or start != 0:
        raise ValueError("Encoded stream requires integer duration ticks and a zero start")
    try:
        time_base = Fraction(stream.get("time_base", ""))
        movie_duration = Fraction(probe.get("format", {}).get("duration", ""))
    except (ValueError, TypeError, ZeroDivisionError) as error:
        raise ValueError("Encoded stream/container timing is malformed") from error
    expected = Fraction(count, fps)
    if time_base <= 0 or ticks * time_base != expected:
        raise ValueError("Encoded stream duration does not exactly match the captured-state hold plan")
    # The qualified FFmpeg MP4 muxer uses a 1000 Hz movie-header clock, distinct
    # from the video track clock. It rounds the movie duration up to whole ticks.
    # For 182/30 seconds this is 6.067 s; the track must still be EXACTLY 182/30.
    # This is an explicit container quantization rule, not a frame-sized tolerance.
    movie_timescale = 1000
    movie_ticks = (expected.numerator * movie_timescale + expected.denominator - 1) // expected.denominator
    if movie_duration != Fraction(movie_ticks, movie_timescale):
        raise ValueError("MP4 movie duration differs from its expected millisecond rounding")


def encode_capture(capture_dir, output_dir, ffmpeg,
                   samples_per_second=4.0, output_fps=30):
    """Create output_dir/movie.mp4 and its verified receipt, returning the Path.

    The output directory must not exist. Failed jobs retain their partial output
    and logs, without a successful manifest. Supplied tools are recorded by hash;
    no system ffmpeg lookup or installation occurs.
    """
    capture = validate_capture(capture_dir)
    boundaries = hold_boundaries(len(capture.frames), samples_per_second, output_fps)
    encoder, probe_tool = _tools(ffmpeg)
    tool_hashes = {"ffmpeg_sha256": sha256_file(encoder),
                   "ffprobe_sha256": sha256_file(probe_tool)}
    destination = Path(output_dir).absolute()
    destination.mkdir()  # Create-only reservation, including failed prior jobs.
    movie = destination / "movie.mp4"
    command = [str(encoder), "-nostdin", "-n", "-threads", "1",
               "-filter_threads", "1", "-f", "image2pipe", "-c:v", "png",
               "-framerate", str(output_fps), "-i", "pipe:0", "-an",
               "-frames:v", str(boundaries[-1]), "-c:v", "libx264", "-threads", "1",
               "-preset", "medium", "-crf", "18", "-pix_fmt", "yuv420p",
               "-movflags", "+faststart", str(movie)]
    _stream_frames(command, capture, boundaries, destination / "encode.log")
    probe_command = [str(probe_tool), "-v", "error", "-threads", "1", "-count_frames",
                     "-show_streams", "-show_format", "-of", "json", str(movie)]
    with (destination / "ffprobe.json").open("xb") as output:
        with (destination / "ffprobe.log").open("xb") as errors:
            subprocess.run(probe_command, check=True, stdout=output, stderr=errors)
    probe_path = destination / "ffprobe.json"
    if probe_path.stat().st_size > 1024 * 1024:
        raise ValueError("Unexpectedly large ffprobe report")
    probe = json.loads(probe_path.read_text(encoding="utf-8"))
    _verify_probe(probe, capture, boundaries[-1], output_fps)
    decode_command = [str(encoder), "-nostdin", "-v", "error", "-xerror", "-threads", "1",
                      "-i", str(movie), "-threads", "1", "-f", "null", "-"]
    with (destination / "decode.log").open("xb") as errors:
        subprocess.run(decode_command, check=True, stdout=subprocess.DEVNULL, stderr=errors)
    final_capture = validate_capture(capture.directory)
    if (final_capture.manifest_sha256 != capture.manifest_sha256
            or final_capture.frame_index_sha256 != capture.frame_index_sha256
            or sha256_file(encoder) != tool_hashes["ffmpeg_sha256"]
            or sha256_file(probe_tool) != tool_hashes["ffprobe_sha256"]):
        raise ValueError("Capture or pinned media tools changed during encoding")
    holds = [{**asdict(frame), "first_video_frame": begin,
              "video_frames": end - begin, "hold_duration_s": (end - begin) / output_fps}
             for frame, begin, end in zip(capture.frames, boundaries, boundaries[1:])]
    receipt = {
        "schema": "robo_dyna.chrono_capture_video.v1",
        "capture": str(capture.directory),
        "capture_manifest_sha256": capture.manifest_sha256,
        "frame_index_sha256": capture.frame_index_sha256,
        "capture_metadata": metadata_dict(capture),
        "video": {"file": movie.name, "bytes": movie.stat().st_size,
                  "sha256": sha256_file(movie), "frames": boundaries[-1],
                  "fps": output_fps, "duration_s": boundaries[-1] / output_fps},
        "samples_per_second": samples_per_second,
        "presentation": "Saved states held at a presentation cadence; physical sample times retained individually.",
        "hold_rule": "Frame boundary i = ceil(i * output_fps / samples_per_second); final state is held too.",
        "recorded_samples": holds,
        "interpolated_frames": False, "deformation_scale": 1,
        "encoding": "Lossy H264/yuv420p; repeated source PNGs, no geometric interpolation or scaling.",
        "full_decode_passed": True, **tool_hashes,
        "encode_command": command, "probe_command": probe_command,
        "decode_command": decode_command,
    }
    _write_json(destination / "manifest.json", receipt)
    return movie


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture")
    parser.add_argument("output")
    parser.add_argument("--ffmpeg", required=True)
    parser.add_argument("--samples-per-second", type=float, default=4.0)
    parser.add_argument("--output-fps", type=int, default=30)
    args = parser.parse_args(argv)
    print(encode_capture(args.capture, args.output, args.ffmpeg,
                         args.samples_per_second, args.output_fps))


if __name__ == "__main__":
    main()
