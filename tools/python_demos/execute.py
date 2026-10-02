"""Execute one unchanged demo with explicit modules/data and its original arguments.

Heavy or interactive launches belong inside the workspace's existing resource
guard. Building or listing a demo never runs this executor.
"""

import argparse
import hashlib
import importlib
import json
import os
from pathlib import Path
import runpy
import sys
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--script", type=Path, required=True)
    parser.add_argument("--package-manifest", type=Path, required=True)
    parser.add_argument("--case", required=True)
    parser.add_argument("--ros-node", type=Path)
    parser.add_argument("--data-root", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args, arguments = parser.parse_known_args()
    if arguments[:1] == ["--"]:
        arguments = arguments[1:]
    script = args.script.absolute()
    package = args.package_manifest.absolute()
    data = args.data_root.resolve(strict=True)
    if not script.is_file() or not package.is_file() or not data.is_dir():
        raise ValueError("The selected script, package manifest and data directory must exist")
    output = args.output_dir.absolute()
    output.mkdir(parents=True, exist_ok=False)
    work = output / "work"
    work.mkdir()
    requested = {"schema": "robodyna.python_demo_launch.v1", "case": args.case,
                 "script": str(script), "script_sha256": hashlib.sha256(script.read_bytes()).hexdigest(),
                 "package_manifest": str(package), "data_root": str(data),
                 "ros_node": str(args.ros_node) if args.ros_node else None,
                 "arguments": arguments, "python": sys.version,
                 "scope": "Original script launch; this receipt alone is not physical, graphics, or GPU qualification."}
    (output / "launch.json").write_text(json.dumps(requested, indent=2) + "\n")
    start = time.monotonic()
    exit_code = 0
    detail = None
    script_started = False
    try:
        if args.ros_node:
            node = args.ros_node.absolute()
            if not node.is_file():
                raise ValueError("Declared ROS node launcher is missing")
            # The unchanged native node loader uses execv on this exact cwd path.
            # The declared launcher supplies its own middleware environment.
            (work / "chrono_ros_node").symlink_to(node)
        sys.path.insert(0, str(package.parent))
        product = importlib.import_module("robodyna")
        product.SetChronoDataPath(str(data) + os.sep)
        product.SetChronoOutputPath(str(output / "output") + os.sep)
        vehicle = getattr(product, "vehicle", None)
        if vehicle is not None:
            vehicle.SetVehicleDataPath(str(data / "vehicle") + os.sep)
        # Preserve direct sibling imports such as FEA's original cables.py.
        sys.path.insert(0, str(script.parent))
        sys.argv = [str(script)] + arguments
        os.chdir(work)
        script_started = True
        runpy.run_path(str(script), run_name="__main__")
    except SystemExit as error:
        exit_code = error.code if isinstance(error.code, int) else 0 if error.code is None else 1
        detail = str(error.code)
        raise
    except BaseException as error:
        exit_code = 130 if isinstance(error, KeyboardInterrupt) else 1
        detail = type(error).__name__ + ": " + str(error)
        raise
    finally:
        (output / "result.json").write_text(json.dumps({
            "schema": "robodyna.python_demo_result.v1", "case": args.case,
            "exit_code": exit_code, "detail": detail, "elapsed_seconds": time.monotonic() - start,
            "script_started": script_started,
            "scope": "Observed original-script process result; no automatic simulation-correctness claim.",
        }, indent=2) + "\n")


if __name__ == "__main__":
    main()
