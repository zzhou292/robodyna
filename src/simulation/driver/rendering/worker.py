"""CPU-only render integrity/encoding stages, always launched through the guard."""

import argparse
from pathlib import Path

from viewer.file_integrity import sha256_file
from viewer.video.encode import encode_capture, hold_boundaries
from ..jsonio import read_json, require, write_new
from .integrity import inventory


def check_tools(tools):
    for name, pin in tools.items():
        path = Path(pin["path"])
        require(path.is_file() and ("bytes" not in pin or path.stat().st_size == pin["bytes"])
                and sha256_file(path) == pin["sha256"],
                f"pinned tool changed: {name}")


def capture_identity(metadata, expected):
    require(metadata.get("schema") == "robo_dyna.physical_replay_capture.v1"
            and metadata.get("complete_capture") is True
            and metadata.get("input_receipt", {}).get("sha256") == expected["viewer_input_sha256"]
            and metadata.get("input_archive_manifest", {}).get("sha256") == expected["archive_manifest_sha256"]
            and type(metadata.get("frames")) is int and metadata["frames"] == expected["saved_states"]
            and metadata.get("final_epoch") == expected["accepted_intervals"]
            and metadata.get("final_time_s") == expected["actual_time_s"]
            and metadata.get("input_horizon_complete") is expected["horizon_complete"]
            and metadata.get("surface_color_mode") == "part-id" and metadata.get("part_palette_seed") == 2
            and metadata.get("deformation_scale") == 1 and metadata.get("interpolated_frames") is False
            and metadata.get("simulation_executed_by_viewer") is False,
            "capture does not represent the selected accepted archive at physical scale")


def encode(request, view):
    require(view in request["views"], "unknown planned view")
    output = Path(request["output"])
    capture = output / (view + "-capture")
    tools = request["tools"]
    check_tools(tools)
    capture_identity(read_json(capture / "manifest.json", 2 << 20), request["input"])
    resources = request["resources"]
    movie = encode_capture(capture, output / (view + "-video"), tools["ffmpeg"]["path"],
                           resources["samples_per_second"], resources["output_fps"])
    metadata = read_json(movie.parent / "manifest.json", 2 << 20)
    capture_identity(metadata["capture_metadata"], request["input"])
    boundaries = hold_boundaries(request["input"]["saved_states"], resources["samples_per_second"], resources["output_fps"])
    require(metadata.get("full_decode_passed") is True and metadata["video"]["frames"] == boundaries[-1]
            and metadata["video"]["fps"] == resources["output_fps"]
            and metadata["video"]["duration_s"] == boundaries[-1] / resources["output_fps"]
            and metadata.get("ffmpeg_sha256") == tools["ffmpeg"]["sha256"]
            and metadata.get("ffprobe_sha256") == tools["ffprobe"]["sha256"], "encoded video differs from planned states/tools")
    check_tools(tools)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("operation", choices=("inventory-before", "inventory-after", "encode"))
    parser.add_argument("request")
    parser.add_argument("--request-sha256", required=True)
    parser.add_argument("--view")
    args = parser.parse_args(argv)
    require(sha256_file(args.request) == args.request_sha256, "render request changed")
    request = read_json(args.request)
    require(request.get("schema") == "robodyna.render_request.v1", "unknown render request schema")
    if args.operation == "encode":
        encode(request, args.view)
    else:
        result = inventory(request["input"]["output"])
        output = Path(request["output"])
        if args.operation == "inventory-after":
            require(result == read_json(output / "inventory-before.json", 16 << 20),
                    "accepted input changed during rendering")
        write_new(output / (args.operation + ".json"), result, 16 << 20)


if __name__ == "__main__":
    main()
