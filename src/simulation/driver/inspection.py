"""Reuse the accepted-result closure contract; never infer a physical restart."""

from pathlib import Path

from viewer.postprocess.lifecycle import closed_run
from .jsonio import read_json, require, resolve


def inspect_run(path, guard=None):
    path = Path(path).absolute()
    if guard is None:
        launch = read_json(path / "launch.json")
        require(launch.get("schema") == "robodyna.launch.v1" and launch.get("mode") == "run",
                "directory has no Robodyna run launch; provide --guard for a legacy accepted directory")
        accepted = resolve(path, launch["output"])
        guard = resolve(path, launch["guard_report"])
    else:
        accepted, guard = path, Path(guard).absolute()
    # Bound JSON before invoking the existing semantic/hash validator. This is a
    # metadata precheck, not the complete C++ all-frame replay verification.
    # Long-run watchdog reports retain sampled telemetry and can exceed the
    # smaller input-schema cap. Keep a distinct bounded reader allowance.
    read_json(guard, 16 << 20)
    for file in (accepted / "summary.json", accepted / "viewer-input.json",
                 accepted / "archive/manifest.json", accepted / "archive/configuration.json",
                 accepted / "archive/frame-index.json"):
        read_json(file)
    summary, index, viewer_hash = closed_run(dict(output=str(accepted), guard_report=str(guard)))
    return dict(schema="robodyna.inspection.v1", output=str(accepted),
                horizon_complete=summary["horizon_complete"], accepted_intervals=summary["accepted_intervals"],
                actual_time_s=summary["actual_completed_time_s"], fixed_dt_s=summary["fixed_dt_s"],
                saved_states=len(index["frames"]), reason=summary["reason"],
                viewer_input_sha256=viewer_hash, physical_restart=False,
                validation_scope="closed_metadata_and_bound_record_hashes",
                full_cpp_replay_verified=False)
