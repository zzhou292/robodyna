"""Tiny transport fixtures, deliberately not physical Yaris cases."""

import hashlib
import json
from pathlib import Path
import zipfile

from src.simulation.driver.manifests import FILE_NAMES, MEMBER_NAMES, GIB, MIB


def write(path, value):
    Path(path).write_text(json.dumps(value))


def digest(data):
    return hashlib.sha256(data).hexdigest()


def case_fixture(root):
    root = Path(root)
    canonical = root / "canonical"
    canonical.mkdir()
    files, members = {}, {}
    for role in FILE_NAMES:
        if role == "source_archive":
            continue
        path = canonical / "manifest.json" if role == "canonical_manifest" else root / (role + ".json")
        data = b'{"fixture":"transport-only"}'
        path.write_bytes(data)
        files[role] = dict(path=str(path.relative_to(root)), bytes=len(data), sha256=digest(data))
    archive = root / "source.zip"
    with zipfile.ZipFile(archive, "w") as target:
        for role in MEMBER_NAMES:
            data = (role + "\n").encode()
            name = "original/" + role + ".key"
            target.writestr(name, data)
            members[role] = dict(member=name, bytes=len(data), sha256=digest(data))
    data = archive.read_bytes()
    files["source_archive"] = dict(path=archive.name, bytes=len(data), sha256=digest(data))
    case = dict(schema="robodyna.case.v1", profile="yaris.native_v6.wall_self", canonical_directory="canonical",
                files=files, original_members=members,
                run=dict(duration_s=.00001515, fixed_dt_s=1.5e-7, samples=3,
                         contact_activity="shell_removal", stage_timing=False,
                         capture_qeph_rejection=True, verify_initial_retry=False))
    resources = dict(schema="robodyna.resources.v1", cpu_threads=4, rss_bytes=18 * GIB,
                     minimum_available_ram_bytes=32 * GIB, gpu_index=0,
                     minimum_gpu_free_bytes=8 * GIB, maximum_gpu_growth_bytes=9 * GIB,
                     timeout_s=600, cooperative_maximum_elapsed_s=540, stop_grace_s=30,
                     archive_bytes=6 * GIB, artifact_file_bytes=24 * MIB,
                     solid_worker_blocks=32, workstation_lock="workstation.lock")
    write(root / "case.json", case)
    write(root / "resources.json", resources)
    return case, resources
