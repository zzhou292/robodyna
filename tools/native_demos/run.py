"""Explicitly run one audited native CPU example; never called by build-all."""

import argparse
import os
from pathlib import Path

from src.simulation.driver.jsonio import read_json
from src.simulation.driver.watchdog import select_watchdog_interpreter
from tools.dependencies.cuda_math import file_hash
from tools.native_demos.cases import prepare_case, validate_build_system_output
from tools.verification.demo_matrix.executor import guarded, save


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--preset", choices=("core_build_system",), required=True)
    parser.add_argument("--environment", type=Path, required=True)
    parser.add_argument("--executable", type=Path, required=True)
    parser.add_argument("--data-root", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    environment = read_json(args.environment)
    if environment.get("schema") != "robodyna.demo_operator_environment.v1":
        raise ValueError("Unsupported operator environment")
    presets = read_json(Path(__file__).with_name("Presets.json"))
    if presets.get("schema") != "robodyna.native_demo_presets.v1":
        raise ValueError("Unsupported native demo preset document")
    preset = presets["presets"][args.preset]
    request = prepare_case(environment["repository"], preset, args.executable, args.data_root, args.output)
    output = args.output.absolute()
    save(output / "request.json", request)
    result = {"schema": "robodyna.native_demo_run.v1", "status": "failed", "target": preset["target"]}
    try:
        interpreter = select_watchdog_interpreter(environment["watchdog"], os.environ,
                                                   explicit=environment.get("watchdog_python"))
        guarded([request["binary"], *request["arguments"]], output, environment, interpreter, 60,
                working_directory=request["working_directory"])
        if file_hash(Path(request["binary"])) != request["binary_sha256"]:
            raise ValueError("Native executable changed during the run")
        result["telemetry"] = validate_build_system_output((output / "build.log").read_text())
        result["status"] = "passed"
    except BaseException as error:
        result["error"] = str(error)
        raise
    finally:
        save(output / "result.json", result)
    print("Original native mechanism completed; evidence:", output)


if __name__ == "__main__":
    main()
