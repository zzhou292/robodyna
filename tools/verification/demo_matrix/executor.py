"""Sequential compile-only controller over existing plans, evidence and guard."""

import fcntl
import json
import os
from pathlib import Path
import signal
import subprocess

from src.simulation.driver.jsonio import read_json, write_new
from src.simulation.driver.runtime import monitored_command, watchdog_environment
from src.simulation.driver.watchdog import select_watchdog_interpreter
from tools.dependencies.cuda_math import file_hash
from .evidence import verify_build
from .inventory import read_inventory, refresh
from .identities import tool_identities
from .plans import batch_targets, coverage, create_plan, disk_cache_option, read_matrix, repository_environment_options
from .phases import prerequisite_plan
from .snapshots import require_unchanged, source_snapshot

JSON_CAP = 16 * 1024**2


def save(path, value):
    write_new(path, value, byte_cap=JSON_CAP)


def guarded(command, directory, environment, interpreter, timeout, working_directory=None):
    """The unchanged shared guard owns resource limits and descendant cleanup."""
    resources = {"workstation_lock": environment["workstation_lock"], "cpu_threads": 8,
                 "rss_bytes": 16 * 1024**3, "minimum_available_ram_bytes": 32 * 1024**3,
                 "timeout_s": timeout}
    guard = Path(environment["watchdog"])
    invocation = monitored_command(guard, resources, directory / "guard.json", command, interpreter["path"])
    child_env = watchdog_environment(os.environ, guard)
    process = None
    def interrupted(signum, _frame):
        raise KeyboardInterrupt("Guarded command interrupted by signal " + str(signum))
    previous_term = signal.signal(signal.SIGTERM, interrupted)
    with (directory / "build.log").open("x") as log:
        try:
            process = subprocess.Popen(invocation, cwd=working_directory or environment["repository"], env=child_env,
                                       stdout=log, stderr=subprocess.STDOUT)
            code = process.wait()
        except BaseException:
            if process is not None and process.poll() is None:
                process.send_signal(signal.SIGTERM)
                process.wait()  # Existing guard handles the authenticated session.
            raise
        finally:
            signal.signal(signal.SIGTERM, previous_term)
    receipt = read_json(directory / "guard.json", byte_cap=JSON_CAP)
    save(directory / "invocation.json", {"schema": "robodyna.guarded_invocation.v1", "command": invocation,
                                         "guard_sha256": file_hash(directory / "guard.json"),
                                         "watchdog_sha256": file_hash(guard), "interpreter": interpreter})
    if code or receipt.get("exit_code") != 0 or receipt.get("status") != "passed" or receipt.get("process_scope", {}).get("cleanup") != "complete":
        raise RuntimeError("Guarded command failed; evidence preserved in " + str(directory))


def next_attempt(parent):
    parent.mkdir(parents=True, exist_ok=True)
    for number in range(1, 10000):
        path = parent / ("attempt-%04d" % number)
        try:
            path.mkdir()
            return path
        except FileExistsError:
            continue
    raise ValueError("Too many attempts in this build root")


def accepted_attempt(parent, batch, inventory, inventory_path, matrix_path):
    for directory in sorted(parent.glob("attempt-*"), reverse=True):
        directory = directory / "outputs"
        accepted = directory / "qualification.json"
        if not accepted.is_file():
            continue
        plan = read_json(directory / "plan.json", byte_cap=JSON_CAP)
        actual = verify_build(plan, directory / "guard.json", directory / "build-events.jsonl")
        if actual != read_json(accepted, byte_cap=JSON_CAP):
            raise ValueError("Completed build receipt changed: " + str(directory))
        if (actual["batch"] != batch["id"] or actual["targets"] != sorted(batch_targets(inventory, batch)) or
                actual["inventory_sha256"] != file_hash(inventory_path) or actual["matrix_sha256"] != file_hash(matrix_path)):
            raise ValueError("Completed build does not match the current admitted batch")
        return directory
    return None


def execute(inventory_matrix, environment_path, output, batch_id=None, resume=False):
    def stop(signum, _frame):
        raise KeyboardInterrupt("Build controller stopped by signal " + str(signum))
    previous = signal.signal(signal.SIGTERM, stop)
    root = Path(output).absolute()
    existed = root.exists()
    try:
        return _execute(inventory_matrix, environment_path, output, batch_id, resume)
    except BaseException as error:
        if (not existed or resume) and (root / "request.json").is_file():
            request = read_json(root / "request.json", byte_cap=JSON_CAP)
            if request.get("schema") == "robodyna.demo_build_request.v1":
                save(next_attempt(root / "controller_failures") / "failure.json",
                     {"schema": "robodyna.demo_build_failure.v1", "error": str(error),
                      "status": "failed", "batch": batch_id})
        raise
    finally:
        signal.signal(signal.SIGTERM, previous)


def _execute(inventory_matrix, environment_path, output, batch_id, resume):
    matrix_path = Path(inventory_matrix).resolve(strict=True)
    matrix = read_matrix(matrix_path)
    environment = read_json(environment_path)
    if environment.get("schema") != "robodyna.demo_operator_environment.v1":
        raise ValueError("Unsupported operator environment")
    disk_cache_option(environment)
    for key in ("repository", "bazel", "output_user_root", "workstation_lock", "watchdog"):
        if not isinstance(environment.get(key), str) or not Path(environment[key]).is_absolute():
            raise ValueError("Operator environment requires absolute " + key)
    if set(environment.get("action_environment", {})) - {"CUDA_PATH", "CUDACXX", "CUDAToolkit_ROOT"}:
        raise ValueError("Unexpected toolchain environment override")
    timeout = environment.get("timeout_seconds", 7200)
    if type(timeout) is not int or not 1 <= timeout <= 43200:
        raise ValueError("Choose a bounded per-batch timeout")
    root = Path(output).absolute()
    repository = Path(environment["repository"]).resolve(strict=True)
    if root == repository or repository in root.parents:
        raise ValueError("Keep build evidence outside the source repository")
    selected = [b for b in matrix["batches"] if batch_id is None or b["id"] == batch_id]
    if not selected:
        raise ValueError("Unknown build batch")
    request = {"schema": "robodyna.demo_build_request.v1", "matrix": matrix,
               "matrix_path": str(matrix_path), "environment": environment, "batch": batch_id}
    if resume:
        if read_json(root / "request.json", byte_cap=JSON_CAP) != request:
            raise ValueError("Resume requires the exact original matrix and environment")
    else:
        root.mkdir(parents=True, exist_ok=False)
        save(root / "request.json", request)
        save(root / "sources.json", source_snapshot(repository))
    # The per-job controller lock prevents two resumptions racing. It never
    # replaces or bypasses the workstation lock used by every guarded command.
    with (root / "controller.lock").open("a") as controller:
        fcntl.flock(controller, fcntl.LOCK_EX | fcntl.LOCK_NB)
        expected = read_json(root / "sources.json", byte_cap=JSON_CAP)
        require_unchanged(repository, expected)
        interpreter = select_watchdog_interpreter(environment["watchdog"], os.environ,
                                                   explicit=environment.get("watchdog_python"))
        identities = tool_identities(environment, interpreter)
        identity_path = root / "tools.json"
        if identity_path.exists():
            if read_json(identity_path, byte_cap=JSON_CAP) != identities:
                raise ValueError("Selected build/guard tools changed; start a fresh build root")
        else:
            save(identity_path, identities)
        inventory_path = root / "inventory.json"
        if not inventory_path.exists():
            attempt = next_attempt(root / "queries")
            options = repository_environment_options(environment)
            query_request = {"repository": str(repository), "bazel_prefix": [environment["bazel"], "--batch",
                             "--output_user_root=" + environment["output_user_root"], "--host_jvm_args=-Xmx2048m"],
                             "repository_options": ["--lockfile_mode=error"] + options}
            action_env = environment.get("action_environment", {})
            if action_env:
                query_request["bazel_prefix"] = ["env"] + [key + "=" + value for key, value in sorted(action_env.items())] + query_request["bazel_prefix"]
            save(attempt / "request.json", query_request)
            command = [interpreter["path"], "-m", "tools.verification.demo_matrix.capture",
                       "--request", str(attempt / "request.json"), "--output", str(attempt / "snapshot")]
            guarded(command, attempt, environment, interpreter, 600)
            snapshot = attempt / "snapshot"
            save(inventory_path, refresh(repository, snapshot / "expanded.xml", snapshot / "public.txt", snapshot / "receipt.json"))
            save(root / "inventory.identity.json", {"sha256": file_hash(inventory_path)})
        if read_json(root / "inventory.identity.json")["sha256"] != file_hash(inventory_path):
            raise ValueError("Recorded inventory changed; refusing resumed build")
        inventory = read_inventory(inventory_path)
        if batch_id is None:
            gaps = coverage(inventory, matrix)
            if any(gaps.values()):
                save(next_attempt(root / "admission") / "gaps.json", gaps)
                raise ValueError("Full build has missing declarations or matrix assignments; see admission evidence")
        # Admit every selected batch's SDKs/configs before the first compilation.
        for batch in selected:
            create_plan(inventory, matrix, batch["id"], environment, inventory_path, matrix_path,
                        root / "preflight-paths" / batch["id"])
        run = next_attempt(root / "runs")
        result = {"schema": "robodyna.demo_build_execution.v1", "status": "failed", "batches": [],
                  "scope": "selected_batch_only" if batch_id else "full_demo_compile_matrix"}
        try:
            for batch in selected:
                require_unchanged(repository, expected)
                if tool_identities(environment, interpreter) != identities:
                    raise ValueError("Build tools changed during this execution")
                parent = root / "batches" / batch["id"]
                completed = accepted_attempt(parent, batch, inventory, inventory_path, matrix_path) if parent.exists() else None
                if completed:
                    print("Re-admitting completed batch through Bazel:", batch["id"], flush=True)
                directory = next_attempt(parent)
                # create_plan admits a new path; it does not create it itself.
                plan = create_plan(inventory, matrix, batch["id"], environment, inventory_path, matrix_path, directory / "outputs")
                prerequisite = prerequisite_plan(plan, batch, directory / "native-prerequisites")
                if prerequisite:
                    pre_output = Path(prerequisite["output_directory"])
                    pre_output.mkdir()
                    save(pre_output / "plan.json", prerequisite)
                    print("Compiling native prerequisites:", batch["id"], "libraries:", len(prerequisite["targets"]), flush=True)
                    guarded(prerequisite["command"], pre_output, environment, interpreter, timeout)
                    require_unchanged(repository, expected)
                    if tool_identities(environment, interpreter) != identities:
                        raise ValueError("Build tools changed while compiling native prerequisites")
                    save(pre_output / "qualification.json", verify_build(prerequisite, pre_output / "guard.json", pre_output / "build-events.jsonl"))
                outputs = Path(plan["output_directory"])
                outputs.mkdir()
                save(outputs / "plan.json", plan)
                print("Compiling batch:", batch["id"], "targets:", len(plan["targets"]), flush=True)
                guarded(plan["command"], outputs, environment, interpreter, timeout)
                require_unchanged(repository, expected)
                if tool_identities(environment, interpreter) != identities:
                    raise ValueError("Build tools changed while compiling the batch")
                save(outputs / "qualification.json", verify_build(plan, outputs / "guard.json", outputs / "build-events.jsonl"))
                result["batches"].append({"id": batch["id"], "status": "passed", "evidence": str(outputs),
                                          "prerequisite_evidence": prerequisite["output_directory"] if prerequisite else None,
                                          "previous_evidence": str(completed) if completed else None})
            result["status"] = "passed"
        except BaseException as error:
            result["error"] = str(error)
            raise
        finally:
            save(run / "summary.json", result)
    return result
