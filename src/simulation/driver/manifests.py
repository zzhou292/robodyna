"""Versioned case and workstation-resource schemas; no solver allocation."""

from dataclasses import dataclass
from pathlib import Path
import re

from .jsonio import boolean, fields, integer, read_json, real, require, resolve
from .sources import member_basename

GIB = 1 << 30
MIB = 1 << 20
PROFILE = "yaris.native_v6.wall_self"
FILE_NAMES = ("source_archive", "canonical_manifest", "scope", "declarations",
              "glass_resolution", "type13", "wall_manifest", "solid_packets")
MEMBER_NAMES = ("member", "auxiliary_member", "original_wall_member", "self_contact_combine_member")


def pin(value, base):
    fields(value, ("path", "bytes", "sha256"))
    require(type(value["sha256"]) is str and re.fullmatch(r"[0-9a-f]{64}", value["sha256"]),
            "source pin requires lowercase SHA-256")
    return dict(path=str(resolve(base, value["path"])),
                bytes=integer(value["bytes"], "source bytes", 1), sha256=value["sha256"])


@dataclass(frozen=True)
class Case:
    manifest: Path
    canonical_directory: Path
    files: dict
    members: dict
    run: dict


def load_case(path):
    path = Path(path).absolute()
    doc = read_json(path)
    fields(doc, ("schema", "profile", "canonical_directory", "files", "original_members", "run"))
    require(doc["schema"] == "robodyna.case.v1" and doc["profile"] == PROFILE,
            "unsupported case schema or profile")
    directory = resolve(path.parent, doc["canonical_directory"])
    fields(doc["files"], FILE_NAMES)
    files = {name: pin(doc["files"][name], path.parent) for name in FILE_NAMES}
    require(Path(files["canonical_manifest"]["path"]).resolve() == (directory / "manifest.json").resolve(),
            "canonical manifest must identify canonical_directory/manifest.json")
    fields(doc["original_members"], MEMBER_NAMES)
    members = {}
    for role in MEMBER_NAMES:
        value = doc["original_members"][role]
        fields(value, ("member", "bytes", "sha256"))
        name = value["member"]
        require(type(name) is str and 0 < len(name) <= 4096 and "\x00" not in name,
                "invalid original archive member")
        identity = pin(dict(path=name, bytes=value["bytes"], sha256=value["sha256"]), Path("/"))
        require(identity["bytes"] <= 64 * MIB, "original member exceeds native 64 MiB limit")
        members[role] = dict(member=name, bytes=identity["bytes"], sha256=identity["sha256"])
    require(len({value["member"] for value in members.values()}) == len(members), "duplicate source member roles")
    require(len({member_basename(value["member"]) for value in members.values()}) == len(members),
            "duplicate original source basenames")
    run = doc["run"]
    fields(run, ("duration_s", "fixed_dt_s", "samples", "contact_activity",
                 "stage_timing", "capture_qeph_rejection", "verify_initial_retry"))
    duration = real(run["duration_s"], "duration_s", high=.1, positive=True)
    dt = real(run["fixed_dt_s"], "fixed_dt_s", positive=True)
    samples = integer(run["samples"], "samples", 2, 1000)
    require(run["contact_activity"] in ("shell_removal", "all_active_prefix"), "unsupported contact activity")
    parsed = dict(duration_s=duration, fixed_dt_s=dt, samples=samples, contact_activity=run["contact_activity"])
    for name in ("stage_timing", "capture_qeph_rejection", "verify_initial_retry"):
        parsed[name] = boolean(run[name], name)
    return Case(path, directory, files, members, parsed)


def load_resources(path):
    path = Path(path).absolute()
    doc = read_json(path)
    names = ("schema", "cpu_threads", "rss_bytes", "minimum_available_ram_bytes", "gpu_index",
             "minimum_gpu_free_bytes", "maximum_gpu_growth_bytes", "timeout_s",
             "cooperative_maximum_elapsed_s", "stop_grace_s", "archive_bytes", "artifact_file_bytes",
             "solid_worker_blocks", "workstation_lock")
    fields(doc, names)
    require(doc["schema"] == "robodyna.resources.v1", "unsupported resource schema")
    result = dict(doc)
    for name in ("cpu_threads", "rss_bytes", "minimum_available_ram_bytes", "minimum_gpu_free_bytes",
                 "maximum_gpu_growth_bytes", "archive_bytes", "artifact_file_bytes", "solid_worker_blocks"):
        result[name] = integer(doc[name], name, 1)
    result["gpu_index"] = integer(doc["gpu_index"], "gpu_index")
    for name in ("timeout_s", "cooperative_maximum_elapsed_s", "stop_grace_s"):
        result[name] = real(doc[name], name, positive=True)
    require(result["cooperative_maximum_elapsed_s"] < result["timeout_s"], "cooperative timeout must precede hard timeout")
    require(result["archive_bytes"] <= 6 * GIB, "archive exceeds native 6 GiB cap")
    require(result["artifact_file_bytes"] <= 32 * MIB, "artifact exceeds native 32 MiB cap")
    require(result["solid_worker_blocks"] in (4, 8, 16, 32), "solid_worker_blocks must be 4, 8, 16 or 32")
    require(result["rss_bytes"] > 2 * MIB, "RSS envelope must include export reservation")
    result["workstation_lock"] = str(resolve(path.parent, doc["workstation_lock"]))
    return result
