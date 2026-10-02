"""Execute one real, local MPI coupon through the existing owned-session guard."""

import argparse
import json
import os
from pathlib import Path
import subprocess
import tempfile

from python.runfiles import runfiles
from viewer.file_integrity import sha256_file
from src.simulation.driver.runtime import monitored_command, watchdog_environment
from src.simulation.driver.watchdog import select_watchdog_interpreter
from tools.mpi.environment import local_command, mpi_environment


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--program", required=True)
    parser.add_argument("--sdk", required=True)
    parser.add_argument("--guard", required=True)
    parser.add_argument("--ranks", required=True, type=int)
    parser.add_argument("--timeout-seconds", type=int, default=120)
    parser.add_argument("--program-input", action="append", default=[])
    args = parser.parse_args()
    if not 1 <= args.timeout_seconds <= 300:
        raise ValueError("MPI coupon timeout must be between one and 300 seconds")
    if not 1 <= args.ranks <= min(8, len(os.sched_getaffinity(0))):
        raise ValueError("MPI rank count exceeds the inherited CPU allowance")
    resolver = runfiles.Create()

    def locate(name):
        value = resolver.Rlocation(name) if resolver else None
        if not value or not Path(value).is_file():
            raise ValueError("Missing declared MPI test input: " + name)
        return Path(value).absolute()

    program, sdk, guard = [locate(value) for value in (args.program, args.sdk, args.guard)]
    program_inputs = [locate(value) for value in args.program_input]
    sdk_record = json.loads(sdk.read_text())
    if sdk_record.get("schema") != "robodyna.mpi_sdk.v1":
        raise ValueError("Unsupported MPI SDK receipt")
    prefix = sdk.parent / "prefix"
    mpirun = prefix / "bin/mpirun.openmpi"
    if not mpirun.is_file() or not os.access(mpirun, os.X_OK):
        raise ValueError("Declared SDK mpirun is absent or not executable")
    output_parent = os.environ.get("TEST_UNDECLARED_OUTPUTS_DIR") or os.environ.get("TEST_TMPDIR")
    output = Path(tempfile.mkdtemp(prefix="robodyna-mpi-", dir=output_parent))
    result = {"schema": "robodyna.local_mpi_test.v1", "status": "failed", "exit_code": None}
    try:
        environment = mpi_environment(os.environ, sdk.parent)
        if not (Path(environment["OMPI_MCA_mca_base_component_path"]) / "mca_schizo_ompi.so").is_file():
            raise ValueError("Declared OpenMPI option-parser component is missing from its actual plugin directory")
        interpreter = select_watchdog_interpreter(guard, environment)
        command = local_command(mpirun, prefix, program, args.ranks) + [str(path) for path in program_inputs]
        request = {"schema": "robodyna.local_mpi_request.v1", "ranks": args.ranks, "threads_per_rank": 1,
                   "sdk_receipt": str(sdk), "sdk_sha256": sha256_file(sdk), "sdk_version": sdk_record["version"],
                   "program": str(program), "program_sha256": sha256_file(program),
                   "program_inputs": [{"path": str(path), "sha256": sha256_file(path)} for path in program_inputs],
                   "mpirun": str(mpirun), "mpirun_sha256": sha256_file(mpirun),
                   "command": command, "watchdog_interpreter": interpreter,
                   "scope": "CPU-only, local ranks, shared-memory MPI; no remote launch"}
        (output / "request.json").write_text(json.dumps(request, indent=2) + "\n")
        guard_report = output / "guard.json"
        resources = {"workstation_lock": str(output / "owned-mpi.lock"), "cpu_threads": args.ranks,
                     "rss_bytes": 4 * 1024**3, "minimum_available_ram_bytes": 32 * 1024**3,
                     "timeout_s": args.timeout_seconds}
        guarded = monitored_command(guard, resources, guard_report, command, interpreter["path"])
        with (output / "test.log").open("x") as log:
            # The guard keeps its mpirun session leader unreaped until all its
            # ordinary descendant process groups are drained using pidfds.
            completed = subprocess.run(guarded, cwd=output, env=watchdog_environment(environment, guard),
                                       stdout=log, stderr=subprocess.STDOUT, check=False)
        result["exit_code"] = completed.returncode
        receipt = json.loads(guard_report.read_text())
        result["guard_sha256"] = sha256_file(guard_report)
        result["cleanup"] = receipt.get("process_scope", {}).get("cleanup")
        if completed.returncode or receipt.get("exit_code") != 0 or result["cleanup"] != "complete":
            raise RuntimeError("Real MPI coupon or owned-session cleanup failed")
        result["status"] = "passed"
    except BaseException as error:
        result["error"] = str(error)
        raise
    finally:
        (output / "result.json").write_text(json.dumps(result, indent=2) + "\n")
        if (output / "test.log").exists():
            print((output / "test.log").read_text())
        print("MPI qualification evidence:", output)


if __name__ == "__main__":
    main()
