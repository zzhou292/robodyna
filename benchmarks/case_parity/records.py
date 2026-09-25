"""Parsed time, output and complete-advancement timing records."""
import math

from .artifacts import keys, require
from .contracts import canonical


def time_grid(pin, parent, contract, artifacts):
    file = artifacts.verify(pin, parent)
    record = artifacts.object(file)
    keys(record, ("schema", "kind", "times_s"), "recorded time grid")
    require(record["schema"] == "robo_dyna.case_time_grid.v1", "unknown time-grid schema")
    definition = contract.domains["recorded_time_grid"]["definition"]
    kind = definition["kind"]
    times = record["times_s"]
    require(record["kind"] == kind and isinstance(times, list) and
            len(times) == definition["planned_steps"] + 1, "time grid does not cover the declared case")
    require(all(type(t) in (int, float) for t in times), "numerical recorded times required")
    require(times[0] == definition["initial_time_s"] and
            times[-1] == definition["requested_end_time_s"], "recorded physical endpoints differ from contract")
    require(all((b > a if kind == "physical_steps" else b >= a)
                for a, b in zip(times, times[1:])), "recorded times are not ordered for their declared scope")
    return file, times


def output_work(pin, parent, contract, artifacts):
    file = artifacts.verify(pin, parent)
    record = artifacts.object(file)
    keys(record, ("schema", "definition", "payloads"), "output work")
    require(record["schema"] == "robo_dyna.case_output_work.v1", "unknown output-work schema")
    expected = contract.domains["output_work_precision"]["definition"]
    require(canonical(record["definition"]) == canonical(expected), "output work differs from declared case")
    names = [f["name"] for f in expected["fields"]]
    require(isinstance(record["payloads"], dict) and set(record["payloads"]) == set(names),
            "output field payload coverage differs")
    for field in expected["fields"]:
        payload = artifacts.verify(record["payloads"][field["name"]], file.path.parent)
        expected_bytes = len(expected["sample_epochs"]) * field["components"] * 8
        require(payload.bytes == expected_bytes, "actual output payload extent differs from declared field work")
    # Numeric values may differ within the separately qualified comparison.
    return canonical(record["definition"])


def warm_timing(pin, parent, contract, producer, completed, elapsed, artifacts):
    file = artifacts.verify(pin, parent)
    record = artifacts.object(file)
    keys(record, ("schema", "contract_sha256", "producer_sha256", "boundary",
                  "first_step", "step_count", "warmup_steps", "total_seconds",
                  "timer_resolution_seconds"), "warm advancement timing")
    require(record["schema"] == "robo_dyna.complete_advancement_timing.v1", "unknown timing-record schema")
    require(record["contract_sha256"] == contract.artifact.sha256 and
            record["producer_sha256"] == producer.sha256, "warm timing is not bound to producer/case")
    expected = ("accepted_complete_step" if contract.scope != "normal_response_packet"
                else "complete_normal_response_update")
    require(record["boundary"] == expected, "partial substage cannot be a complete advancement timing")
    for name in ("first_step", "step_count", "warmup_steps"):
        require(type(record[name]) is int and record[name] > 0, "explicit positive warm window required")
    first, count, warmup = record["first_step"], record["step_count"], record["warmup_steps"]
    require(first > warmup and first + count - 1 <= completed, "warm timing window is outside completed work")
    total, resolution = record["total_seconds"], record["timer_resolution_seconds"]
    require(type(total) in (int, float) and type(resolution) in (int, float) and
            0 < resolution and 1000 * resolution <= total <= elapsed,
            "warm timing is invalid or too short relative to its declared timer resolution")
    return total / count, (record["boundary"], first, count, warmup, resolution)
