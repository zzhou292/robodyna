"""Create-only working directories for explicitly audited native demo presets."""

import math
from pathlib import Path
import re

from tools.dependencies.cuda_math import file_hash


def prepare_case(repository, preset, executable, data_root, output):
    repo = Path(repository).resolve(strict=True)
    source = repo / preset["source"]
    if file_hash(source) != preset["source_sha256"]:
        raise ValueError("Audited native entrypoint changed; review its launch preset")
    binary = Path(executable).resolve(strict=True)
    expected_binary = (repo / preset["binary"]).resolve(strict=True)
    if binary != expected_binary or not binary.is_file():
        raise ValueError("Executable does not identify this preset's actual Bazel target")
    if preset["mode"] != "cpu_headless" or preset["writes_into_data"]:
        raise ValueError("This first launcher admits only audited headless CPU demos that do not write into data")
    data = Path(data_root).resolve(strict=True)
    if not data.is_dir():
        raise ValueError("An explicit existing retained data directory is required")
    requested = Path(output).absolute()
    if requested.exists() or requested.is_symlink():
        raise ValueError("Native demo output must be create-only")
    destination = requested.resolve(strict=False)
    if destination.is_relative_to(repo) or destination.is_relative_to(data):
        raise ValueError("Case output must be outside source and data trees")
    destination.mkdir(parents=True)
    (destination / "work").mkdir()
    (destination / "data").symlink_to(data, target_is_directory=True)
    (destination / "work/DEMO_OUTPUT").mkdir()
    return {"schema": "robodyna.native_demo_request.v1", "target": preset["target"],
            "source": str(source), "source_sha256": preset["source_sha256"],
            "binary": str(binary), "binary_sha256": file_hash(binary), "arguments": preset["arguments"],
            "working_directory": str(destination / "work"), "data_root": str(data),
            "data_identity_scope": "Explicit resolved directory/link only; full transitive asset contents are not hashed or qualified",
            "mode": preset["mode"], "scope": preset["scope"]}


_FRAME = re.compile(r"^Time:\s+(\S+)\s+Steps:\s+(\d+)\s+Slider X position:\s+(\S+)\s+Engine torque:\s+(\S+)\s*$")


def validate_build_system_output(text):
    frames = []
    for line in text.splitlines():
        match = _FRAME.fullmatch(line.strip())
        if match:
            time, steps, position, torque = match.groups()
            frames.append((float(time), int(steps), float(position), float(torque)))
    if len(frames) not in (49, 50) or not 2.44 <= frames[-1][0] <= 2.51:
        raise ValueError("Original mechanism did not report its expected requested frame horizon")
    if any(not all(math.isfinite(value) for value in row) for row in frames):
        raise ValueError("Native mechanism reported non-finite state")
    if any(a[0] >= b[0] or a[1] >= b[1] for a, b in zip(frames, frames[1:])):
        raise ValueError("Native frame requests/solver-step counters did not advance")
    positions = [row[2] for row in frames]
    if frames[-1][1] < 200 or max(positions) - min(positions) <= 0.1:
        raise ValueError("Original mechanism did not produce meaningful motion and solver steps")
    return {"reported_frames": len(frames), "last_requested_frame_time_s": frames[-1][0],
            "reported_solver_steps": frames[-1][1], "slider_x_range": max(positions) - min(positions),
            "scope": "Original stdout frame requests, actual solver-step counts and slider/torque values; requested frame time is not an independently sampled System clock"}
