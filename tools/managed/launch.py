"""Run a real managed demo in a create-only output directory with declared DSOs."""

import argparse
import json
import os
from pathlib import Path
import subprocess
import time

from python.runfiles import runfiles
from tools.managed.environment import mono_environment


def resolve_inputs(arguments):
    resolver = runfiles.Create()
    if resolver is None:
        raise RuntimeError("Managed launch requires declared Bazel runfiles")
    result = {}
    for name in ("mono", "assembly", "managed", "native_wrapper", "native_core", "data_anchor"):
        value = resolver.Rlocation(getattr(arguments, name))
        if not value or not Path(value).is_file():
            raise RuntimeError(f"Declared managed input is missing: {name}")
        result[name] = Path(value).absolute()
    return result


def mapped_backend(pid):
    """Observe actual loaded backend paths; do not infer this from link metadata."""
    try:
        lines = Path(f"/proc/{pid}/maps").read_text().splitlines()
    except (FileNotFoundError, ProcessLookupError):
        return set()
    return {line.split()[-1] for line in lines
            if line.split()[-1].endswith("/librobodyna_core.so")}


def run_demo(inputs, output, timeout=30):
    output = Path(output).absolute()
    output.mkdir(parents=True, exist_ok=False)
    result = {"schema": "robodyna.managed_demo_run.v1", "status": "running",
              "inputs": {name: str(path) for name, path in inputs.items()},
              "timeout_seconds": timeout, "native_backend_paths": [],
              "scope": "Managed execution of the original demo against the existing native backend"}
    (output / "request.json").write_text(json.dumps(result, indent=2) + "\n")
    seen = set()
    started = time.monotonic()
    process = None
    try:
        sdk_root = inputs["mono"].parents[2]
        native_directories = sorted({inputs["native_wrapper"].parent, inputs["native_core"].parent})
        environment = mono_environment(os.environ, sdk_root, [inputs["managed"].parent], native_directories)
        # The unchanged configured helper refers to data/. Never use the operator's
        # working directory or a globally installed data tree implicitly.
        (output / "data").symlink_to(inputs["data_anchor"].parent, target_is_directory=True)
        command = [str(inputs["mono"]), "--config", str(sdk_root / "etc/mono/config"), str(inputs["assembly"])]
        with (output / "stdout.log").open("x") as log:
            process = subprocess.Popen(command, cwd=output, env=environment, stdout=log, stderr=subprocess.STDOUT)
            while process.poll() is None:
                seen.update(mapped_backend(process.pid))
                if time.monotonic() - started > timeout:
                    raise TimeoutError("Managed demo exceeded its bounded launch duration")
                time.sleep(0.02)
            result["exit_code"] = process.wait()
            result["status"] = "complete" if result["exit_code"] == 0 else "failed"
    except BaseException as error:
        if process is not None and process.poll() is None:
            process.kill()
            process.wait()
        result.update(status="failed", error=str(error), exit_code=process.returncode if process else None)
        raise
    finally:
        result["native_backend_paths"] = sorted(seen)
        result["elapsed_seconds"] = time.monotonic() - started
        (output / "result.json").write_text(json.dumps(result, indent=2) + "\n")
    if result["exit_code"]:
        raise RuntimeError(f"Managed demo failed; see {output / 'stdout.log'}")
    return result


def argument_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("mono", "assembly", "managed", "native-wrapper", "native-core", "data-anchor"):
        parser.add_argument("--" + name, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--timeout", type=float, default=30)
    return parser


def main():
    args = argument_parser().parse_args()
    if not 0 < args.timeout <= 300:
        raise ValueError("Managed demo timeout must be positive and at most300 seconds")
    result = run_demo(resolve_inputs(args), args.output, args.timeout)
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
