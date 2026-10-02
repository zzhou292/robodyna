"""Read the limited source-list subset of the two retained Vehicle CMake files.

This is not a CMake configure evaluator. Unknown conditions in a selected source
declaration reject instead of silently changing the admitted implementation.
"""

from pathlib import Path
import posixpath
import re
import shlex

from tools.verification.cmake_evidence import commands

PROFILE = {
    "CH_ENABLE_MODULE_FEA": True,
    "CH_ENABLE_MODULE_VSG": True,
    "CH_ENABLE_MODULE_IRRLICHT": True,
    "CH_USE_OPENCRG": False,
    "CH_ENABLE_MODULE_FSI_SPH": False,
    "CH_USE_SCM_GPU": False,
}


def _condition(value, profile):
    if value in profile:
        return profile[value]
    if value.startswith("else of (") and value.endswith(")"):
        return not _condition(value[9:-1], profile)
    raise ValueError("Unreviewed Vehicle source condition: " + value)


def source_groups(path, source_directory, prefix, profile=PROFILE):
    groups = {}
    rows = commands(Path(path).read_text())
    for row in rows:
        tokens = shlex.split(row["arguments"])
        if row["command"] == "set" and tokens:
            name, values, append = tokens[0], tokens[1:], False
        elif row["command"] == "list" and len(tokens) >= 2 and tokens[0] == "APPEND":
            name, values, append = tokens[1], tokens[2:], True
        else:
            continue
        if not name.startswith(prefix) or not name.endswith("_FILES"):
            continue
        if not all(_condition(condition, profile) for condition in row["conditions"]):
            continue
        expanded = list(groups.get(name, [])) if append else []
        for value in values:
            match = re.fullmatch(r"\$\{([A-Za-z0-9_]+)\}", value)
            if match:
                if match[1] not in groups:
                    raise ValueError("Unresolved Vehicle source group: " + value)
                expanded.extend(groups[match[1]])
            elif value.startswith("${CMAKE_SOURCE_DIR}/"):
                expanded.append(value.removeprefix("${CMAKE_SOURCE_DIR}/"))
            elif value:
                if "$" in value:
                    raise ValueError("Unreviewed Vehicle source expression: " + value)
                expanded.append(posixpath.normpath(source_directory + "/" + value))
        groups[name] = expanded
    return groups


def library_groups(path, library, groups):
    """Return only final add_library groups, not their overlapping helper groups."""
    for row in commands(Path(path).read_text()):
        tokens = shlex.split(row["arguments"])
        if row["command"] != "add_library" or not tokens or tokens[0] != library:
            continue
        result = {}
        for value in tokens[1:]:
            match = re.fullmatch(r"\$\{([A-Za-z0-9_]+)\}", value)
            if not match:
                raise ValueError("Unreviewed Vehicle library operand: " + value)
            name = match[1]
            if name == "CV_CONFIG_FILE":
                continue  # Reuse the already owned generated SCM/Vehicle header.
            if name not in groups:
                raise ValueError("Missing final Vehicle source group: " + name)
            # CMake repeats CV_WV_BRAKE_FILES in the final list. It still compiles
            # each source once; preserve that unique source ownership here.
            result[name] = groups[name]
        return result
    raise ValueError("Retained library declaration not found: " + library)
