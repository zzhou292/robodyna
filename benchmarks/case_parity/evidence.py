"""Bind actual named numerical tests and producer output to guarded executions."""
import re

from output.gtest_evidence import require_completed_test
from .artifacts import keys, require, text
from .contracts import canonical
from .records import time_grid, output_work, warm_timing

_ASSIGNMENT = re.compile(r"[A-Za-z_][A-Za-z0-9_]*=.*\Z", re.S)
_NAME = re.compile(r"[A-Za-z_][A-Za-z0-9_]*\Z")


def invocation(command, producer):
    require(isinstance(command, list) and command and all(isinstance(x, str) for x in command),
            "guard command is missing")
    offset = 0
    if command[0] in ("env", "/usr/bin/env"):
        offset = 1
        while offset < len(command):
            if _ASSIGNMENT.fullmatch(command[offset]):
                offset += 1
            elif command[offset] == "-u":
                require(offset + 1 < len(command) and _NAME.fullmatch(command[offset + 1]),
                        "invalid env unset operation")
                offset += 2
            else:
                break
    require(offset < len(command) and command[offset] == str(producer.path),
            "guard did not directly invoke the pinned producer")
    args = command[offset + 1:]
    require("--forecast-only" not in args, "forecast-only execution is not a completed benchmark")
    return args


def option(args, name):
    require(args.count(name) == 1 and args.index(name) + 1 < len(args), f"unique {name} binding required")
    return args[args.index(name) + 1]


def passed_guard(pin, parent, artifacts):
    file = artifacts.verify(pin, parent)
    guard = artifacts.object(file)
    require(type(guard.get("exit_code")) is int and guard["exit_code"] == 0 and
            guard.get("status") == "passed" and
            guard.get("process_scope", {}).get("cleanup") == "complete",
            "evidence guard did not pass and clean up")
    return file, guard


def numerical_evidence(pin, parent, reference, candidate, timing_pairs, artifacts):
    file = artifacts.verify(pin, parent)
    value = artifacts.object(file)
    keys(value, ("schema", "reference_contract_sha256", "candidate_contract_sha256",
                 "reference_producer", "candidate_producer", "comparator", "timing_manifest",
                 "xml", "guard"), "numerical evidence")
    require(value["schema"] == "robo_dyna.case_numerical_evidence.v1", "unknown numerical evidence schema")
    require(value["reference_contract_sha256"] == reference.artifact.sha256 and
            value["candidate_contract_sha256"] == candidate.artifact.sha256,
            "numerical evidence belongs to different contracts")
    require(reference.numerical_protocol is not None and candidate.numerical_protocol is not None,
            "numerical comparison protocol is unqualified")
    producers = {name: artifacts.verify(value[name], file.path.parent)
                 for name in ("reference_producer", "candidate_producer", "comparator")}
    manifest = artifacts.verify(value["timing_manifest"], file.path.parent)
    recorded = artifacts.object(manifest)
    keys(recorded, ("schema", "pairs"), "numerically checked timing manifest")
    require(recorded["schema"] == "robo_dyna.case_timing_manifest.v1" and
            canonical(recorded["pairs"]) == canonical(timing_pairs),
            "numerical evidence did not inspect these measured runs")
    # Resolve pins relative to the manifest, then expose absolute paths to the caller.
    resolved_pairs = []
    for pair in timing_pairs:
        keys(pair, ("order", "reference", "candidate"), "timing pair")
        resolved = dict(pair)
        for name in ("reference", "candidate"):
            actual = artifacts.verify(pair[name], manifest.path.parent)
            resolved[name] = {"path": str(actual.path), "sha256": actual.sha256}
        resolved_pairs.append(resolved)
    xml = artifacts.verify(value["xml"], file.path.parent)
    _, guard = passed_guard(value["guard"], file.path.parent, artifacts)
    expected = reference.numerical_protocol["test_name"]
    args = invocation(guard.get("command"), producers["comparator"])
    require("--gtest_filter=" + expected in args and "--gtest_output=xml:" + str(xml.path) in args,
            "numerical guard invoked a different test/report")
    required = {
        "reference_contract_sha256": reference.artifact.sha256,
        "candidate_contract_sha256": candidate.artifact.sha256,
        "reference_producer_sha256": producers["reference_producer"].sha256,
        "candidate_producer_sha256": producers["candidate_producer"].sha256,
        "numerical_protocol": reference.numerical_protocol["id"],
        "tolerances_sha256": reference.numerical_protocol["tolerances"]["sha256"],
        "timing_manifest_sha256": manifest.sha256,
    }
    require_completed_test(xml.path, expected, required)
    return producers, resolved_pairs


def measured_run(pin, parent, contract, producer, artifacts):
    file = artifacts.verify(pin, parent)
    wrapper = artifacts.object(file)
    keys(wrapper, ("schema", "guard", "producer_record", "launcher_interval"), "guarded measurement")
    require(wrapper["schema"] == "robo_dyna.guarded_case_measurement.v1", "unknown measurement schema")
    record_file = artifacts.verify(wrapper["producer_record"], file.path.parent)
    value = artifacts.object(record_file)
    keys(value, ("schema", "run_id", "backend", "contract_sha256", "producer_sha256",
                 "requested_steps", "completed_steps", "start_time_s", "end_time_s",
                 "time_grid", "output_work", "timing_scope", "warm_timing",
                 "platform_id"), "completed producer record")
    require(value["schema"] == "robo_dyna.completed_case_measurement.v1", "unknown producer-record schema")
    text(value["run_id"], "run identity")
    require(value["contract_sha256"] == contract.artifact.sha256 and
            value["producer_sha256"] == producer.sha256, "measurement producer/case pin differs")
    guard_file, guard = passed_guard(wrapper["guard"], file.path.parent, artifacts)
    args = invocation(guard.get("command"), producer)
    require(option(args, "--run-id") == value["run_id"] and
            option(args, "--benchmark-record") == str(record_file.path),
            "executed producer is not bound to this completed record")
    time_definition = contract.domains["recorded_time_grid"]["definition"]
    steps = time_definition["planned_steps"]
    require(type(value["requested_steps"]) is int and type(value["completed_steps"]) is int and
            value["completed_steps"] == value["requested_steps"] == steps,
            "incomplete or shorter work cannot satisfy the declared contract")
    grid, times = time_grid(value["time_grid"], record_file.path.parent, contract, artifacts)
    require(value["start_time_s"] == times[0] and value["end_time_s"] == times[-1],
            "producer endpoint fields disagree with the parsed time grid")
    require(value["timing_scope"] == "startup_advance_and_equivalent_output",
            "solver-only or component timing cannot be relabelled end-to-end")
    elapsed = guard.get("elapsed_seconds")
    require(type(elapsed) in (int, float) and elapsed > 0, "positive measured guard elapsed required")
    definition = output_work(value["output_work"], record_file.path.parent, contract, artifacts)
    mean, window = warm_timing(value["warm_timing"], record_file.path.parent, contract, producer,
                               steps, elapsed, artifacts)
    text(value["platform_id"], "machine/boot identity")
    interval_file = artifacts.verify(wrapper["launcher_interval"], file.path.parent)
    interval = artifacts.object(interval_file)
    keys(interval, ("schema", "clock", "platform_id", "guard_sha256",
                    "producer_record_sha256", "start_ns", "end_ns"), "launcher interval")
    require(interval["schema"] == "robo_dyna.benchmark_launcher_interval.v1" and
            interval["clock"] == "CLOCK_MONOTONIC_NS", "unsupported launcher timing clock")
    require(interval["platform_id"] == value["platform_id"] and
            interval["guard_sha256"] == guard_file.sha256 and
            interval["producer_record_sha256"] == record_file.sha256,
            "launcher interval belongs to different executed evidence")
    require(type(interval["start_ns"]) is int and type(interval["end_ns"]) is int and
            0 <= interval["start_ns"] < interval["end_ns"] < (1 << 63),
            "invalid complete launcher interval")
    complete_elapsed = (interval["end_ns"] - interval["start_ns"]) * 1e-9
    # run_bounded rounds elapsed to milliseconds; the launcher encloses its
    # Popen/wait in ONE clock, including preflight and cleanup.
    require(elapsed <= complete_elapsed + .001, "guard elapsed exceeds its bound launcher span")
    result = dict(value)
    result.update(guard_sha256=guard_file.sha256, elapsed_seconds=complete_elapsed,
                  warm_step_seconds=mean, warm_window=window,
                  time_grid_sha256=grid.sha256, output_definition=definition,
                  start_ns=interval["start_ns"], end_ns=interval["end_ns"], affinity=guard.get("cpu_affinity"),
                  host_cpus=guard.get("limits", {}).get("cpus"))
    return result
