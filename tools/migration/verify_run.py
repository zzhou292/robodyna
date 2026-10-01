"""Compare a completed product run against a pinned accepted baseline.

Read-only for both runs. Closure receipts are required before archive access.
From the source checkout, expose the retained Python helpers with
PYTHONPATH=src/compatibility/app and run ``python3 -B -m tools.migration.verify_run``.
This compares recorded output, not unsaved solver state or CPU-reference accuracy.
"""

import argparse
from pathlib import Path, PurePosixPath
import re

from output.json_object import read_object
from viewer.file_integrity import sha256_file
from src.simulation.driver.inspection import inspect_run
from src.simulation.driver.job import verify_completion
from src.simulation.driver.jsonio import read_json, require, write_new
from src.simulation.driver.receipts import verify_product_records


TIMING_FIELDS = frozenset({
    "session_startup_s", "runtime_elapsed_s", "runtime_steps_per_s",
    "step_s", "commit_s", "archive_s", "capture_s",
})


def record_path(root, name):
    path = PurePosixPath(name)
    require(not path.is_absolute() and path.parts and ".." not in path.parts and str(path) == name,
            "archive record path must be canonical and relative")
    result = root.joinpath(*path.parts)
    require(result.resolve().is_relative_to(root.resolve()), "archive record escapes its root")
    require(result.is_file() and not result.is_symlink(), f"regular archive record required: {name}")
    return result


def authenticate_inventory(accepted):
    """Hash every declared file and reject undeclared, missing or symlink entries."""
    root = Path(accepted) / "archive"
    require(root.is_dir() and not root.is_symlink(), "regular closed archive directory required")
    manifest = read_json(root / "manifest.json")
    declared = {}
    for row in manifest["files"]:
        require(type(row) is dict and set(row) == {"file", "bytes", "sha256"}, "malformed archive inventory record")
        name = row["file"]
        require(type(name) is str and name not in declared and name != "manifest.json", "duplicate or invalid archive record")
        require(type(row["bytes"]) is int and row["bytes"] > 0 and
                type(row["sha256"]) is str and re.fullmatch(r"[0-9a-f]{64}", row["sha256"]),
                "malformed archive extent or digest")
        declared[name] = row
    actual = set()
    for path in root.rglob("*"):
        require(not path.is_symlink(), f"archive symlink is not a payload: {path}")
        require(path.is_file() or path.is_dir(), f"nonregular archive entry: {path}")
        if path.is_file():
            actual.add(path.relative_to(root).as_posix())
    require(actual == set(declared) | {"manifest.json"}, "closed archive file inventory differs")
    observed = {}
    for name, row in declared.items():
        path = record_path(root, name)
        size = path.stat().st_size
        require(size == row["bytes"], f"archive record extent differs: {name}")
        digest = sha256_file(path)
        require(digest == row["sha256"], f"archive record hash differs: {name}")
        observed[name] = {"bytes": size, "sha256": digest}
    manifest_path = root / "manifest.json"
    observed["manifest.json"] = {"bytes": manifest_path.stat().st_size, "sha256": sha256_file(manifest_path)}
    return observed


def summary_differences(baseline, candidate):
    old, old_raw = read_object(Path(baseline) / "summary.json", max_bytes=1 << 20)
    new, new_raw = read_object(Path(candidate) / "summary.json", max_bytes=1 << 20)
    require(old.keys() == new.keys(), "accepted summary field inventory differs")
    differences = sorted(key for key in old_raw if key not in TIMING_FIELDS and old_raw[key] != new_raw[key])
    timings = sorted(key for key in TIMING_FIELDS if old_raw.get(key) != new_raw.get(key))
    return differences, timings


def require_closed_guard(path):
    guard = read_json(path, 16 << 20)
    require(type(guard.get("exit_code")) is int and guard["exit_code"] == 0 and
            guard.get("process_scope", {}).get("cleanup") == "complete",
            "comparison requires a completed zero-exit guard; active/stopped output is not admitted")


def compare_run(baseline, baseline_guard, candidate_run, baseline_manifest_sha256, intervals=101):
    baseline, candidate_run = Path(baseline), Path(candidate_run)
    candidate = candidate_run / "accepted"
    # These receipts are final publications. Never inspect accepted files first.
    require_closed_guard(baseline_guard)
    require_closed_guard(candidate_run / "guard.json")
    result = read_json(candidate_run / "launch-result.json")
    require(result.get("schema") == "robodyna.launch_result.v1" and result.get("mode") == "run" and
            type(result.get("return_code")) is int and result["return_code"] == 0 and
            result.get("product_result_verified") is True,
            "candidate lacks a verified completed product launch")
    verify_product_records(candidate_run, result, "run")
    native = read_json(candidate_run / "native-report.json")
    verify_completion(candidate_run, "run", 0, native)
    old_info = inspect_run(baseline, baseline_guard)
    new_info = inspect_run(candidate_run)
    require(old_info["horizon_complete"] and new_info["horizon_complete"] and
            old_info["accepted_intervals"] == new_info["accepted_intervals"] == intervals,
            "comparison horizons differ or are incomplete")
    require(re.fullmatch(r"[0-9a-f]{64}", baseline_manifest_sha256) and
            sha256_file(baseline / "archive/manifest.json") == baseline_manifest_sha256,
            "baseline differs from the independently qualified archive pin")
    before = authenticate_inventory(baseline)
    after = authenticate_inventory(candidate)
    changed = sorted(name for name in before.keys() | after.keys() if before.get(name) != after.get(name))
    summary_changed, timing_changed = summary_differences(baseline, candidate)
    viewer_exact = sha256_file(baseline / "viewer-input.json") == sha256_file(candidate / "viewer-input.json")
    old_index = read_json(baseline / "archive/frame-index.json")
    new_index = read_json(candidate / "archive/frame-index.json")
    old_config = read_json(baseline / "archive/configuration.json")
    new_config = read_json(candidate / "archive/configuration.json")
    passed = not changed and not summary_changed and viewer_exact
    return {
        "schema": "robodyna.migration.accepted_run_comparison.v1", "passed": passed,
        "baseline": str(baseline.absolute()), "candidate": str(candidate.absolute()),
        "accepted_intervals": intervals,
        "baseline_fixed_dt_s": old_config["fixed_dt_s"], "candidate_fixed_dt_s": new_config["fixed_dt_s"],
        "baseline_saved_epochs": [frame["stamp"]["epoch"] for frame in old_index["frames"]],
        "candidate_saved_epochs": [frame["stamp"]["epoch"] for frame in new_index["frames"]],
        "baseline_archive_files": len(before), "candidate_archive_files": len(after),
        "baseline_archive_bytes": sum(record["bytes"] for record in before.values()),
        "candidate_archive_bytes": sum(record["bytes"] for record in after.values()),
        "all_archive_files_authenticated": True, "all_archive_files_exact": not changed,
        "archive_differences": changed, "viewer_receipt_exact": viewer_exact,
        "nontiming_summary_original_json_exact": not summary_changed,
        "summary_differences": summary_changed, "timing_differences": timing_changed,
        "candidate_full_cpp_replay_verified": native["full_cpp_replay_verified"],
        "driver_reports_compared": False,
        "sha256_and_bytes_by_archive_path": after,
        "claim_boundary": "Recorded accepted physics/static output regression; no unsaved-state, restart, longer-horizon or CPU-reference accuracy claim.",
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline", type=Path, required=True)
    parser.add_argument("--baseline-guard", type=Path, required=True)
    parser.add_argument("--candidate-run", type=Path, required=True)
    parser.add_argument("--baseline-manifest-sha256", required=True)
    parser.add_argument("--intervals", type=int, default=101)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    require(args.intervals > 0, "positive comparison interval count required")
    require(not args.output.exists() and not args.output.is_symlink(), "comparison receipt must be new")
    require(not args.output.resolve().is_relative_to(args.baseline.resolve()) and
            not args.output.resolve().is_relative_to(args.candidate_run.resolve()),
            "comparison receipt must be outside both preserved runs")
    try:
        result = compare_run(args.baseline, args.baseline_guard, args.candidate_run,
                             args.baseline_manifest_sha256, args.intervals)
    except (ValueError, OSError, KeyError) as error:
        result = {"schema": "robodyna.migration.accepted_run_comparison.v1", "passed": False, "error": str(error)}
    write_new(args.output, result)
    print("comparison passed" if result["passed"] else "comparison failed; inspect " + str(args.output))
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
