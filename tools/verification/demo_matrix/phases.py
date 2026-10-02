"""Native prerequisites and safe wrapper compilation share the same profile."""

import copy
from pathlib import Path
import re

_TARGET = re.compile(r"^//[A-Za-z0-9_./+-]*:[A-Za-z0-9_./+-]+$")


def prerequisite_plan(demo_plan, batch, output_directory):
    targets = batch.get("prerequisite_targets", [])
    if not targets:
        return None
    if len(targets) != len(set(targets)) or any(not _TARGET.fullmatch(target) for target in targets):
        raise ValueError("Prerequisites require unique explicit native-library labels")
    if set(targets) & set(demo_plan["targets"]):
        raise ValueError("A native prerequisite cannot also be counted as a demo target")
    output = Path(output_directory).absolute()
    if output.exists():
        raise ValueError("Prerequisite evidence directory must be create-only")
    result = copy.deepcopy(demo_plan)
    result.update(phase="native_prerequisites", targets=sorted(targets), source_by_target={},
                  output_directory=str(output),
                  scope="Native implementation prerequisites only; these libraries do not count as retained demo programs")
    command = result["command"][:-len(demo_plan["targets"])]
    if result["command"][-len(demo_plan["targets"]):] != demo_plan["targets"]:
        raise ValueError("Unexpected compile plan target placement")
    job_options = [i for i, value in enumerate(command) if value.startswith("--jobs=")]
    event_options = [i for i, value in enumerate(command) if value.startswith("--build_event_json_file=")]
    if len(job_options) != 1 or len(event_options) != 1:
        raise ValueError("Prerequisite phase requires one worker limit and BEP output")
    command[job_options[0]] = "--jobs=4"
    command[event_options[0]] = "--build_event_json_file=" + str(output / "build-events.jsonl")
    result["command"] = command + result["targets"]
    result["resources"]["compiler_workers"] = 4
    return result
