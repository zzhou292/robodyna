"""Create one guarded product launch without executing test frameworks."""

from pathlib import Path
import subprocess

from viewer.file_integrity import sha256_file
from .jsonio import read_json, require, write_new
from .manifests import load_case, load_resources, PROFILE
from .runtime import clean_environment, guard_command, runtime_file, watchdog_environment
from .sources import extract_members, verify_sources
from .receipts import product_record_names, record
from .watchdog import select_watchdog_interpreter


def verify_completion(output, mode, return_code, native):
    """A zero process exit alone cannot establish preparation or simulation."""
    report = read_json(output / "guard.json", 16 << 20)
    require(type(report.get("exit_code")) is int and report["exit_code"] == return_code
            and report.get("process_scope", {}).get("cleanup") == "complete",
            "watchdog receipt lacks matching exit and complete cleanup")
    require(type(native) is dict and native.get("schema") == "robodyna.native_vehicle_driver.v1"
            and native.get("mode") == mode and native.get("profile") == PROFILE
            and type(native.get("exit_code")) is int and native["exit_code"] == return_code
            and native.get("physics_prepared") is True,
            "native backend did not publish the requested product result")
    if mode == "plan":
        require(return_code == 0 and native.get("fits_runtime_limits") is True
                and native.get("physical_owner_created") is False and native.get("horizon_complete") is False
                and not (output / "accepted").exists(), "plan did not close without creating a physical run")
    else:
        require(native.get("full_cpp_replay_verified") is True and native.get("valid_closed_archive") is True
                and native.get("horizon_complete") is (return_code == 0), "native run did not publish a verified closed result")
        from .inspection import inspect_run
        inspected = inspect_run(output)
        require(inspected["accepted_intervals"] == native.get("accepted_intervals")
                and inspected["actual_time_s"] == native.get("actual_time_s"), "native report and accepted archive differ")


def launch(case_path, resource_path, destination, mode, backend=None, guard=None, watchdog_python=None):
    require(mode in ("plan", "run"), "unsupported native execution mode")
    case, resources = load_case(case_path), load_resources(resource_path)
    verify_sources(case)
    backend = runtime_file("_main/apps/cli/robodyna_native", backend)
    guard = runtime_file("legacy_fea/tools/run_bounded.py", guard)
    require(backend.stat().st_mode & 0o111, "native backend must be executable")
    environment = watchdog_environment(clean_environment(resources["gpu_index"]), guard)
    interpreter = select_watchdog_interpreter(guard, environment, watchdog_python)
    output = Path(destination).absolute()
    require(not output.resolve().is_relative_to(case.canonical_directory.resolve()),
            "output must be outside the immutable canonical source directory")
    require(not output.exists() and not output.is_symlink(), "output must be a new directory")
    require(output.parent.is_dir(), "output parent must already exist")
    output.mkdir()
    members = extract_members(case, output / "sources")
    paths = {name: value["path"] for name, value in case.files.items()
             if name not in ("source_archive", "canonical_manifest", "solid_packets")}
    paths.update(members, canonical=str(case.canonical_directory))
    request = dict(schema="robodyna.native_vehicle_request.v1", profile=PROFILE,
                   source_paths=paths, solid_packets=case.files["solid_packets"], run=case.run,
                   resources=resources, output=str(output / "accepted"),
                   report=str(output / "native-report.json"), stop_file=str(output / "stop.requested"))
    request_path = output / "request.json"
    write_new(request_path, request)
    command = guard_command(guard, backend, request_path, sha256_file(request_path), resources, output, mode, interpreter["path"])
    launch_record = dict(schema="robodyna.launch.v1", mode=mode, profile=PROFILE,
                         output="accepted", guard_report="guard.json", request="request.json",
                         request_sha256=sha256_file(request_path),
                         case_manifest=dict(path=str(case.manifest), sha256=sha256_file(case.manifest)),
                         resource_manifest=dict(path=str(Path(resource_path).absolute()), sha256=sha256_file(resource_path)),
                         native_backend=dict(path=str(backend), sha256=sha256_file(backend)),
                         watchdog=dict(path=str(guard), sha256=sha256_file(guard)),
                         watchdog_interpreter=interpreter,
                         physics_prepared=False, command=command)
    write_new(output / "launch.json", launch_record)
    print(f"{mode}: source preparation uses GPU {resources['gpu_index']}; log: {output / 'native.log'}", flush=True)
    with (output / "native.log").open("xb") as log:
        completed = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT,
                                   env=environment, check=False)
    code = completed.returncode
    result = dict(schema="robodyna.launch_result.v1", mode=mode, return_code=code,
                  guard_report=str(output / "guard.json"), output=str(output))
    if (output / "native-report.json").is_file():
        result["native"] = read_json(output / "native-report.json")
    if code in (0, 2):
        try:
            verify_completion(output, mode, code, result.get("native"))
            result["product_result_verified"] = True
            result["records"] = {name: record(output, name) for name in product_record_names(mode)}
        except (ValueError, OSError, KeyError) as error:
            result["verification_error"] = str(error)
            result["product_result_verified"] = False
            code = 1
    result["return_code"] = code
    write_new(output / "launch-result.json", result)
    return result, code
