"""Accept explicit completed-target build evidence without inferring runtime success."""

import json
from pathlib import Path

from tools.verification.chrono_inventory import digest
from .query import canonical_label


def completed_targets(path):
    passed, skipped = set(), set()
    with Path(path).open() as stream:
        for line in stream:
            if len(line) > 16 * 1024**2:
                raise ValueError("Oversized BEP event")
            record = json.loads(line)
            identity = record.get("id", {})
            if "targetSkipped" in identity:
                skipped.add(canonical_label(identity["targetSkipped"]["label"]))
            target = identity.get("targetCompleted")
            if target and not target.get("aspect") and record.get("completed", {}).get("success"):
                passed.add(canonical_label(target["label"]))
    return passed, skipped


def verify_build(plan, guard_path, bep_path):
    guard = json.loads(Path(guard_path).read_text())
    if guard.get("status") != "passed" or guard.get("exit_code") != 0 or guard.get("process_scope", {}).get("cleanup") != "complete":
        raise ValueError("Build guard did not close successfully with complete cleanup")
    if guard.get("command") != plan["command"]:
        raise ValueError("Build receipt does not execute the exact planned command")
    limits, wanted = guard.get("limits", {}), plan["resources"]
    if (limits.get("cpus", float("inf")) > wanted["affinity_cpus"] or
            limits.get("max_rss_gib", float("inf")) > wanted["rss_gib"] or
            limits.get("min_available_gib", 0) < wanted["minimum_available_ram_gib"] or
            limits.get("timeout_seconds", float("inf")) > wanted["timeout_seconds"]):
        raise ValueError("Actual guard resource limits exceed the admitted plan")
    invocation = json.loads(Path(guard_path).with_name("invocation.json").read_text())
    if invocation.get("schema") != "robodyna.guarded_invocation.v1" or invocation.get("guard_sha256") != digest(Path(guard_path).read_bytes()):
        raise ValueError("Guard invocation receipt does not bind this completed report")
    command = invocation["command"]
    if "--lock" not in command or command[command.index("--lock") + 1] != wanted["workstation_lock"]:
        raise ValueError("Build did not use the admitted workstation lock")
    if "--" not in command or command[command.index("--") + 1:] != plan["command"]:
        raise ValueError("Guard invocation does not match the planned build")
    passed, skipped = completed_targets(bep_path)
    expected = set(plan["targets"])
    if expected & skipped or not expected.issubset(passed):
        raise ValueError("Every explicitly planned target must complete; skipped/missing targets are not a pass")
    return {"schema": "robodyna.demo_build_evidence.v1", "status": "passed",
            "batch": plan["batch"], "phase": plan.get("phase", "demo_targets"), "targets": sorted(expected), "configs": plan["configs"],
            "inventory_sha256": plan["inventory_sha256"], "matrix_sha256": plan["matrix_sha256"],
            "guard_sha256": digest(Path(guard_path).read_bytes()), "bep_sha256": digest(Path(bep_path).read_bytes()),
            "invocation_sha256": digest(Path(guard_path).with_name("invocation.json").read_bytes()),
            "scope": "Explicit target compilation under the named plan/profile; no demo runtime, numerical or GPU execution claim"}
