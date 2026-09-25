"""Conservative paired variation gate after complete case and numerical admission."""
import statistics

from .artifacts import Artifacts, keys, require
from .contracts import read_contract, compare_contracts
from .evidence import numerical_evidence, measured_run


def assess(request_pin, parent):
    artifacts = Artifacts()
    request_file = artifacts.verify(request_pin, parent)
    request = artifacts.object(request_file)
    keys(request, ("schema", "reference", "candidate", "numerics", "timing_pairs"), "comparison request")
    require(request["schema"] == "robo_dyna.case_parity_request.v1", "unknown request schema")
    require(isinstance(request["timing_pairs"], list) and len(request["timing_pairs"]) <= 16,
            "timing pair list exceeds cap")
    require(request["numerics"] is None or isinstance(request["numerics"], dict),
            "numerical evidence must be a pinned artifact, never a pass boolean")
    base = request_file.path.parent
    reference = read_contract(request["reference"], base, artifacts)
    candidate = read_contract(request["candidate"], base, artifacts)
    mismatches = compare_contracts(reference, candidate)
    report = {
        "schema": "robo_dyna.case_parity_assessment.v1",
        "status": "incomparable" if mismatches else "numerics_unqualified",
        "case_scope": candidate.scope,
        "reference_case": reference.case_id, "candidate_case": candidate.case_id,
        "mismatches": mismatches, "gpu_speedup": None, "full_vehicle_requirement_met": False,
        "trust_boundary": "Evidence consistency only. Pinned declarations and a completed named test do not independently prove scientific truth or performance generality.",
    }
    if mismatches or request["numerics"] is None or reference.numerical_protocol is None or candidate.numerical_protocol is None:
        artifacts.recheck()
        report["evidence_files"] = artifacts.inventory()
        return report
    producers, pairs = numerical_evidence(request["numerics"], base, reference, candidate,
                                         request["timing_pairs"], artifacts)
    report["status"] = "comparable_without_speed_win"
    require(isinstance(pairs, list) and len(pairs) <= 16, "timing pair list exceeds cap")
    if len(pairs) < 3:
        report["performance_reason"] = "at_least_three_interleaved_pairs_required"
        artifacts.recheck(); report["evidence_files"] = artifacts.inventory()
        return report
    rows = []
    identities = set()
    guards = set()
    work_identity = None
    previous_end = None
    for index, pair in enumerate(pairs):
        keys(pair, ("order", "reference", "candidate"), "timing pair")
        require(pair["order"] == ("reference_candidate" if index % 2 == 0 else "candidate_reference"),
                "timing protocol must alternate paired execution order")
        a = measured_run(pair["reference"], base, reference, producers["reference_producer"], artifacts)
        b = measured_run(pair["candidate"], base, candidate, producers["candidate_producer"], artifacts)
        require(a["backend"] == "cpu_reference" and b["backend"] == "cuda_candidate", "wrong recorded backend roles")
        for run in (a, b):
            require(run["run_id"] not in identities and run["guard_sha256"] not in guards,
                    "duplicate timing run/receipt cannot count as a repeat")
            identities.add(run["run_id"]); guards.add(run["guard_sha256"])
        for key in ("requested_steps", "completed_steps", "start_time_s", "end_time_s",
                    "time_grid_sha256", "output_definition", "host_cpus", "affinity",
                    "platform_id", "warm_window"):
            require(a[key] == b[key] and a[key] is not None, f"measured work/resource mismatch: {key}")
        require(type(a["host_cpus"]) is int and a["host_cpus"] > 0 and a["affinity"],
                "actual host resource metadata required")
        identity = tuple(str(a[k]) for k in ("requested_steps", "completed_steps",
            "start_time_s", "end_time_s", "time_grid_sha256", "output_definition",
            "host_cpus", "affinity", "platform_id", "warm_window"))
        require(work_identity is None or identity == work_identity,
                "replicates changed work, hardware or resources")
        work_identity = identity
        ordered = (a, b) if index % 2 == 0 else (b, a)
        for run in ordered:
            require(previous_end is None or run["start_ns"] >= previous_end,
                    "timing runs overlap or contradict declared interleaving")
            previous_end = run["end_ns"]
        rows.append((a, b))
    # Full separation of observed ranges is conservative; it is not a statistical confidence interval.
    ratios = {}
    faster = True
    for name, key in (("end_to_end", "elapsed_seconds"), ("warm_step", "warm_step_seconds")):
        a = [pair[0][key] for pair in rows]; b = [pair[1][key] for pair in rows]
        ratios[name] = {
            "reference_median_seconds": statistics.median(a),
            "candidate_median_seconds": statistics.median(b),
            "paired_speedup_min": min(x/y for x, y in zip(a, b)),
            "paired_speedup_median": statistics.median(x/y for x, y in zip(a, b)),
            "range_separation_lower_bound": min(a)/max(b),
        }
        faster &= max(b) < min(a)
    report["gpu_speedup"] = ratios
    report["performance_reason"] = "separated_observed_ranges" if faster else "no_repeatable_advancement_and_end_to_end_win"
    if faster:
        report["status"] = "measured_gpu_win"
        report["full_vehicle_requirement_met"] = candidate.scope == "full_vehicle"
    report["variation_scope"] = "Observed paired/range bound, not a population confidence interval; no extrapolation beyond the declared case."
    artifacts.recheck()
    report["evidence_files"] = artifacts.inventory()
    return report
