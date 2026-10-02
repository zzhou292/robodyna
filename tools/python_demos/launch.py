"""Resolve declared demo/package inputs and invoke the admitted CPython SDK."""

import argparse
import os
import subprocess
import sys
from python.runfiles import runfiles


def main():
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--interpreter", required=True)
    parser.add_argument("--executor", required=True)
    parser.add_argument("--script", required=True)
    parser.add_argument("--package-manifest", required=True)
    parser.add_argument("--case", required=True)
    parser.add_argument("--ros-node")
    parser.add_argument("--robodyna-data-root", required=True)
    parser.add_argument("--robodyna-output-dir", required=True)
    args, script_arguments = parser.parse_known_args()
    resolver = runfiles.Create()
    resolved = [resolver.Rlocation(value) for value in
                (args.interpreter, args.executor, args.script, args.package_manifest)]
    if not all(resolved):
        raise RuntimeError("Declared Robodyna Python demo inputs are missing")
    interpreter, executor, script, package = resolved
    if script_arguments[:1] == ["--"]:
        script_arguments = script_arguments[1:]
    node_arguments = []
    if args.ros_node:
        node = resolver.Rlocation(args.ros_node)
        if not node:
            raise RuntimeError("Declared ROS subprocess launcher is missing")
        node_arguments = ["--ros-node", node]
    environment = dict(os.environ)
    environment.pop("PYTHONPATH", None)
    environment.pop("PYTHONHOME", None)
    command = [interpreter, "-I", "-B", "-X", "faulthandler", executor,
               "--script", script, "--package-manifest", package, "--case", args.case,
               "--data-root", args.robodyna_data_root, "--output-dir", args.robodyna_output_dir,
               ] + node_arguments + ["--"] + script_arguments
    raise SystemExit(subprocess.call(command, env=environment))


if __name__ == "__main__":
    main()
