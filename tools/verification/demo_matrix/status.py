"""Read closed per-batch build evidence, including an active controller's progress."""

from pathlib import Path

from src.simulation.driver.jsonio import read_json
from .evidence import verify_build


def build_status(build_root):
    root = Path(build_root)
    targets = set()
    batches = {}
    # A qualification is published only after the guard and every explicit
    # target pass. Final controller summaries are not needed to observe it.
    # Prerequisite and in-progress directories do not match this file pattern.
    for qualification in sorted(root.glob("batches/*/attempt-*/outputs/qualification.json")):
        directory = qualification.parent
        if not directory.resolve().is_relative_to(root.resolve()):
            raise ValueError("Build evidence points outside its admitted root")
        plan = read_json(directory / "plan.json", byte_cap=16 * 1024**2)
        actual = verify_build(plan, directory / "guard.json", directory / "build-events.jsonl")
        if actual != read_json(qualification, byte_cap=16 * 1024**2):
            raise ValueError("Recorded compilation evidence changed")
        batch = directory.parent.parent.name
        if actual["batch"] != batch or actual.get("phase") != "demo_targets":
            raise ValueError("Closed evidence does not identify this demo batch")
        batches[batch] = {"status": "last_successful_compilation", "evidence": str(directory), "targets": len(actual["targets"])}
        targets.update(actual["targets"])
    return {"batches": batches, "compiled_labels": sorted(targets),
            "scope": "Validated closed per-batch historical compilation evidence, including progress before controller completion. A later failed or in-progress attempt does not erase the last pass. Build-all --resume re-admits SDKs and restores outputs through Bazel; this status does not assert current artifact presence, overall controller success or runtime qualification."}
