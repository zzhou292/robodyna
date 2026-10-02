"""Read retained Sensor source declarations for one explicit Linux profile.

This limited source-list reader is not a general CMake evaluator. Unknown source
conditions and unresolved group references reject admission.
"""

from pathlib import Path
import re
import shlex

from tools.verification.cmake_evidence import commands

PROFILE = {
    "CH_USE_SENSOR_OPTIX": False,
    "CH_USE_SENSOR_VULKAN_RT": True,
    "CH_USE_SENSOR_VULKAN_RT_GPU": True,
    "CH_USE_SENSOR_METAL_RT": False,
    "CH_USE_SENSOR_NVDB": False,
    "CH_ENABLE_MODULE_FSI_SPH": False,
}


def condition(expression, profile=PROFILE):
    if expression.startswith("else of (") and expression.endswith(")"):
        return not condition(expression[9:-1], profile)
    if " OR " in expression:
        return any([condition(part, profile) for part in expression.split(" OR ")])
    if " AND " in expression:
        return all([condition(part, profile) for part in expression.split(" AND ")])
    if expression.startswith("NOT "):
        return not condition(expression[4:], profile)
    if expression not in profile:
        raise ValueError("Unreviewed Sensor source condition: " + expression)
    return profile[expression]


def source_groups(path, profile=PROFILE):
    groups = {}
    for row in commands(Path(path).read_text()):
        tokens = shlex.split(row["arguments"])
        if row["command"] == "set" and tokens:
            name, values, append = tokens[0], tokens[1:], False
        elif row["command"] == "list" and len(tokens) > 1 and tokens[0] == "APPEND":
            name, values, append = tokens[1], tokens[2:], True
        else:
            continue
        if not name.startswith("Chrono_sensor_") or not name.endswith(("_SOURCES", "_HEADERS")):
            continue
        if not all([condition(expression, profile) for expression in row["conditions"]]):
            continue
        result = list(groups.get(name, [])) if append else []
        for value in values:
            reference = re.fullmatch(r"\$\{(\w+)\}", value)
            if reference:
                if reference[1] not in groups:
                    raise ValueError("Unresolved Sensor source group: " + value)
                result.extend(groups[reference[1]])
            elif value:
                if "$" in value or value.startswith("/") or ".." in Path(value).parts:
                    raise ValueError("Unreviewed Sensor source expression: " + value)
                result.append("src/chrono_sensor/" + value)
        groups[name] = result
    return groups
