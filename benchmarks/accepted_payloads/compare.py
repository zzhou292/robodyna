"""Bounded-memory comparison of two closed vehicle throughput archives."""
import hashlib
from pathlib import Path

from raw_json import read_object


TIMING_FIELDS = frozenset({
    "startup_wall_s", "elapsed_after_startup_s", "accepted_intervals_per_second",
    "successful_prepare_wall_s", "successful_commit_wall_s",
    "successful_archive_wall_s", "successful_accepted_capture_wall_s",
    "mechanics_stage_timing",
})
REQUIRED_FIELDS = frozenset({
    "schema", "session_initialized", "valid_archive_manifest", "accepted_intervals",
    "complete_host_upper_bound", "complete_archive_upper_bound", "accepted_mechanics",
    "sampled_shell_plasticity", "archive_manifest_sha256", "viewer_input_sha256",
    "successful_prepare_wall_s", "elapsed_after_startup_s", "accepted_intervals_per_second",
})


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def inventory(directory):
    if not directory.is_dir() or directory.is_symlink():
        raise ValueError(f"regular archive directory required: {directory}")
    result = {}
    for path in sorted(directory.rglob("*")):
        if path.is_symlink():
            raise ValueError(f"archive symlink is not a payload: {path}")
        if path.is_file():
            result[path.relative_to(directory).as_posix()] = sha256(path)
        elif not path.is_dir():
            raise ValueError(f"nonregular archive entry: {path}")
    if "manifest.json" not in result:
        raise ValueError("closed archive manifest required")
    return result


def compare(baseline, candidate, expected_host_delta=0):
    baseline, candidate = Path(baseline), Path(candidate)
    old, old_raw = read_object(baseline / "run-summary.json")
    new, new_raw = read_object(candidate / "run-summary.json")
    for value in (old, new):
        if (not REQUIRED_FIELDS.issubset(value)
                or value.get("schema") != "robo_dyna.vehicle_run_summary.v1"
                or value.get("session_initialized") is not True
                or value.get("valid_archive_manifest") is not True
                or value.get("accepted_intervals", 0) <= 0):
            raise ValueError("initialized accepted run with closed archive required")
    if old.keys() != new.keys():
        raise ValueError("summary field inventory differs")
    excluded = TIMING_FIELDS | {"complete_host_upper_bound"}
    different = sorted(k for k in old if k not in excluded and old_raw[k] != new_raw[k])
    if different:
        raise ValueError(f"original summary values differ: {different}")
    delta = new["complete_host_upper_bound"] - old["complete_host_upper_bound"]
    if delta != expected_host_delta:
        raise ValueError(f"host forecast delta {delta} != declared {expected_host_delta}")
    before, after = inventory(baseline / "archive"), inventory(candidate / "archive")
    if before != after:
        changed = sorted(k for k in before.keys() | after.keys() if before.get(k) != after.get(k))
        raise ValueError(f"archive inventory or payload differs: {changed}")
    viewer = sha256(baseline / "viewer-input.json")
    if viewer != sha256(candidate / "viewer-input.json"):
        raise ValueError("viewer receipt differs")
    if (before["manifest.json"] != old["archive_manifest_sha256"]
            or viewer != old["viewer_input_sha256"]):
        raise ValueError("summary digest does not authenticate archive/viewer")
    return {
        "schema": "robo_dyna.accepted_payload_comparison.v2",
        "baseline": str(baseline.resolve()), "candidate": str(candidate.resolve()),
        "archive_files": len(after),
        "binary_files": sum(name.endswith(".bin") for name in after),
        "archive_files_exact": True, "viewer_receipt_exact": True,
        "all_nontiming_summary_original_json_exact": True,
        "accepted_mechanics_original_json_exact": True,
        "sampled_shell_plasticity_original_json_exact": True,
        "host_forecast_delta": delta, "archive_forecast_exact": True,
        "baseline_prepare_wall_s": old["successful_prepare_wall_s"],
        "candidate_prepare_wall_s": new["successful_prepare_wall_s"],
        "baseline_after_startup_s": old["elapsed_after_startup_s"],
        "candidate_after_startup_s": new["elapsed_after_startup_s"],
        "baseline_steps_per_s": old["accepted_intervals_per_second"],
        "candidate_steps_per_s": new["accepted_intervals_per_second"],
        "sha256_by_archive_path": after,
    }
