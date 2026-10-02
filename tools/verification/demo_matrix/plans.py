"""Readable compile batches with explicit profiles, SDKs and actual target labels."""

import fnmatch
import json
from pathlib import Path
import re

from tools.verification.chrono_inventory import digest
from .roster import build_metadata
from .phases import prerequisite_plan


def compiler_workers(batch):
    workers = batch.get("compiler_workers", 4)
    if type(workers) is not int or not 1 <= workers <= 4:
        raise ValueError("Compiler workers must be an integer from one through four")
    if set(batch.get("families", [])) & {"python_demo", "csharp_demo"} and workers > 2:
        raise ValueError("Language wrapper batches admit one or two compiler workers")
    return workers


def read_matrix(path):
    value = json.loads(Path(path).read_text())
    if value.get("schema") != "robodyna.demo_build_matrix.v1":
        raise ValueError("Unsupported demo compile matrix")
    ids = [b["id"] for b in value["batches"]]
    if len(ids) != len(set(ids)):
        raise ValueError("Duplicate build batch")
    for batch in value["batches"]:
        compiler_workers(batch)
    return value


def disk_cache_option(environment):
    """Use Bazel's own optional disk cache without creating or managing it."""
    if "disk_cache" not in environment:
        return []
    value = environment["disk_cache"]
    if not isinstance(value, str) or not Path(value).is_absolute():
        raise ValueError("disk_cache must be an absolute directory path")
    path = Path(value)
    if path.exists() and not path.is_dir():
        raise ValueError("disk_cache identifies a file, not a directory")
    return ["--disk_cache=" + value]


def repository_environment_options(environment, required=()):
    """Keep supplied repository identities stable while profiles select owners.

    Bazel can revisit previously discovered headers when reusing the output base
    across profiles. Removing an unused SDK variable would invalidate that local
    repository before the current action can discard its old header inputs.
    Supplying its path does not add the SDK to the configured dependency graph.
    """
    values = environment.get("sdk_env", {})
    for name in sorted(required):
        value = values.get(name, "")
        if not isinstance(value, str) or not Path(value).is_absolute() or not Path(value).exists():
            raise ValueError("Missing declared SDK input " + name)
    for name, value in values.items():
        if (not re.fullmatch(r"[A-Z][A-Z0-9_]*", name) or not isinstance(value, str) or
                not Path(value).is_absolute() or not Path(value).exists()):
            raise ValueError("Invalid supplied SDK input " + str(name))
    return ["--repo_env=" + name + "=" + value for name, value in sorted(values.items())]


def batch_targets(inventory, batch):
    targets = {}
    for row in inventory["programs"]:
        if row["family"] not in batch["families"]:
            continue
        for target in row["public_targets"]:
            label = target["label"]
            if not any(fnmatch.fnmatchcase(label, pattern) for pattern in batch["targets"]):
                continue
            if any(fnmatch.fnmatchcase(label, pattern) for pattern in batch.get("exclude", [])):
                continue
            targets[label] = row["source"]
    return targets


def coverage(inventory, matrix):
    assigned = {source for batch in matrix["batches"] for source in batch_targets(inventory, batch).values()}
    return {"declared_but_unassigned": [r["source"] for r in inventory["programs"] if r["public_targets"] and r["source"] not in assigned],
            "not_declared": [r["source"] for r in inventory["programs"] if not r["public_targets"]]}


def create_plan(inventory, matrix, batch_id, environment, inventory_path, matrix_path, output_directory):
    if environment.get("schema") != "robodyna.demo_operator_environment.v1":
        raise ValueError("Unsupported operator environment")
    batch = next((row for row in matrix["batches"] if row["id"] == batch_id), None)
    if batch is None:
        raise ValueError("Unknown build batch: " + batch_id)
    workers = compiler_workers(batch)
    targets = batch_targets(inventory, batch)
    if not targets:
        raise ValueError("Batch has no queried public executable targets")
    repository = Path(environment["repository"]).resolve(strict=True)
    if not inventory.get("captured_build_metadata") or inventory["captured_build_metadata"] != build_metadata(repository):
        raise ValueError("Capture fresh paired Bazel queries for the current build declarations before planning")
    for name, expected in inventory["input_sha256"].items():
        if digest((repository / name).read_bytes()) != expected:
            raise ValueError("Discovery manifest changed; refresh the inventory: " + name)
    for row in inventory["programs"]:
        if row["source"] in targets.values() and digest((repository / row["source"]).read_bytes()) != row["current_sha256"]:
            raise ValueError("Program source changed; refresh inventory: " + row["source"])
    sdk_names = set(batch.get("sdk_env", []))
    for group in batch.get("sdk_sets", []):
        sdk_names.update(matrix["sdk_sets"][group])
    repository_options = repository_environment_options(environment, sdk_names)
    bazel = Path(environment["bazel"])
    if not bazel.is_absolute() or not bazel.is_file():
        raise ValueError("Bazel must be an explicit executable path")
    configs = list(batch["configs"])
    if batch.get("cuda", False):
        architecture = environment.get("cuda_arch")
        if architecture not in matrix["cuda_arch_configs"]:
            raise ValueError("Select an admitted CUDA architecture config")
        configs = ["cuda", architecture] + configs
    unknown = set(configs) - set(matrix["known_configs"])
    if unknown:
        raise ValueError("Unadmitted configuration names: " + str(sorted(unknown)))
    expanded = set(configs)
    for config in configs:
        expanded.update(matrix.get("config_implies", {}).get(config, []))
    for row in inventory["programs"]:
        if row["source"] in targets.values() and not set(row["required_profiles"]).issubset(expanded):
            raise ValueError("Batch omits the recorded required profile for " + row["source"])
    output = Path(output_directory).absolute()
    if output.exists():
        raise ValueError("Build evidence directory must be create-only")
    command = [str(bazel), "--batch", "--output_user_root=" + environment["output_user_root"],
               "--host_jvm_args=-Xmx2048m", "build", "--keep_going",
               "--jobs=" + str(workers), "--lockfile_mode=error",
               "--build_event_json_file=" + str(output / "build-events.jsonl")]
    command += disk_cache_option(environment)
    command += ["--config=" + name for name in configs]
    command += repository_options
    command += sorted(targets)
    action_environment = environment.get("action_environment", {})
    if set(action_environment) - {"CUDA_PATH", "CUDACXX", "CUDAToolkit_ROOT"}:
        raise ValueError("Only the declared CUDA toolchain environment belongs in a compile plan")
    if action_environment:
        command = ["env"] + [key + "=" + value for key, value in sorted(action_environment.items())] + command
    result = {"schema": "robodyna.demo_compile_plan.v1", "batch": batch_id, "phase": "demo_targets", "configs": configs,
            "targets": sorted(targets), "source_by_target": targets,
            "inventory_sha256": digest(Path(inventory_path).read_bytes()),
            "matrix_sha256": digest(Path(matrix_path).read_bytes()), "command": command,
            "repository": str(repository), "command_environment": action_environment,
            "required_sdk_env": sorted(sdk_names), "supplied_sdk_env": sorted(environment.get("sdk_env", {})),
            "resources": {"affinity_cpus": 8, "compiler_workers": workers,
                          "rss_gib": 16, "minimum_available_ram_gib": 32,
                          "workstation_lock": environment["workstation_lock"],
                          "timeout_seconds": environment.get("timeout_seconds", 7200)},
            "output_directory": str(output),
            "scope": "Compile only through the shared workstation guard; no simulation, GUI, MPI or GPU execution"}
    prerequisite_plan(result, batch, output / "prerequisite-admission")
    return result
