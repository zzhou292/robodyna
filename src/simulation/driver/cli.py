"""Small operator interface; mechanics remain in the normal native backend."""

import argparse
import json
import sys
import zipfile

from .inspection import inspect_run
from .job import launch
from .manifests import load_case, load_resources
from .sources import verify_sources


def parser():
    command = argparse.ArgumentParser(prog="robodyna", description="Prepare and run qualified Robodyna cases.")
    sub = command.add_subparsers(dest="command", required=True)
    validate = sub.add_parser("validate", help="check manifest and source identities; no GPU or physics preparation")
    validate.add_argument("case")
    validate.add_argument("--resources")
    for name in ("plan", "run"):
        item = sub.add_parser(name, help="guarded native preparation" if name == "plan" else "guarded native simulation")
        item.add_argument("case")
        item.add_argument("--resources", required=True)
        item.add_argument("--output", required=True, help="new output directory; parent must exist")
        item.add_argument("--backend", help="explicit normal native backend, otherwise use packaged runfiles")
        item.add_argument("--guard", help="explicit existing watchdog, otherwise use packaged runfiles")
        item.add_argument("--watchdog-python", help="explicit Linux Python with pidfd/waitid monitor support")
    inspect = sub.add_parser("inspect", help="check closed run metadata and source-bound receipt hashes")
    inspect.add_argument("run")
    inspect.add_argument("--producer-guard", "--guard", dest="guard", help="legacy guard JSON; run then names the accepted directory")
    render = sub.add_parser("render", help="render a closed accepted archive through the existing viewer and encoder")
    render.add_argument("run")
    render.add_argument("--resources", required=True, help="render resource JSON, separate from simulation limits")
    render.add_argument("--tools", required=True, help="pinned viewer and ffmpeg/ffprobe tool JSON")
    render.add_argument("--output", required=True, help="new directory outside the accepted archive")
    render.add_argument("--view", action="append", choices=("overview", "front"), help="default: both views")
    render.add_argument("--producer-guard", help="required for legacy accepted directories")
    render.add_argument("--guard", help="explicit watchdog script instead of packaged runfiles")
    render.add_argument("--watchdog-python", help="explicit Linux Python with pidfd/waitid monitor support")
    return command


def main(argv=None):
    args = parser().parse_args(argv)
    try:
        code = 0
        if args.command == "validate":
            result = verify_sources(load_case(args.case))
            if args.resources:
                load_resources(args.resources)
                result["resource_schema_valid"] = True
        elif args.command == "inspect":
            result = inspect_run(args.run, args.guard)
        elif args.command == "render":
            from .rendering.pipeline import render
            result = render(args.run, args.resources, args.tools, args.output, args.view,
                            args.producer_guard, args.guard, args.watchdog_python)
        else:
            result, code = launch(args.case, args.resources, args.output, args.command, args.backend, args.guard, args.watchdog_python)
        print(json.dumps(result, indent=2, sort_keys=True, allow_nan=False))
        return code
    except (ValueError, OSError, KeyError, TypeError, RuntimeError, zipfile.BadZipFile) as error:
        print(f"robodyna: {error}", file=sys.stderr)
        return 1
