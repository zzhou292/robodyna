"""Stream first-party source/build inputs for honest build-resume admission."""

from pathlib import Path

from tools.dependencies.cuda_math import file_hash
from tools.verification.chrono_inventory import git

SOURCE_SUFFIXES = frozenset((".c", ".cc", ".cpp", ".cxx", ".cu", ".cuh", ".h", ".hh", ".hpp", ".hxx",
                             ".i", ".py", ".cs", ".bzl", ".cmake", ".in", ".fbs", ".idl", ".json", ".yaml", ".yml",
                             ".S", ".s", ".sh", ".bash", ".bat", ".ps1",
                             ".glsl", ".vert", ".frag", ".comp", ".geom", ".tesc", ".tese",
                             ".rgen", ".rchit", ".rmiss", ".rahit", ".rint", ".rcall", ".metal", ".ptx", ".asm",
                             ".C", ".m", ".mm", ".f", ".for", ".f90", ".f95", ".msg", ".srv", ".action"))


def source_snapshot(repository):
    root = Path(repository)
    names = set(git(root, "ls-files", "--cached", "--others", "--exclude-standard", "-z").decode().split("\0"))
    result = {}
    for name in sorted(names):
        if not name or name.startswith("docs/verification/"):
            continue  # Historical output evidence is not executable source.
        path = root / name
        if path.suffix not in SOURCE_SUFFIXES and path.name not in ("BUILD", "BUILD.bazel", "MODULE.bazel", "MODULE.bazel.lock", "CMakeLists.txt", ".bazelrc", ".bazelversion", ".bazelignore"):
            continue
        if path.is_file():
            result[name] = file_hash(path)
    return {"schema": "robodyna.demo_source_snapshot.v1", "files": result,
            "scope": "Git-visible source/build/schema/configuration inputs; runtime archives, media and binary model assets are not qualified by compilation"}


def require_unchanged(repository, expected):
    actual = source_snapshot(repository)
    if actual != expected:
        changed = sorted(name for name in actual["files"].keys() | expected["files"].keys()
                         if actual["files"].get(name) != expected["files"].get(name))
        raise ValueError("Source/build inputs changed; start a fresh build root: " + ", ".join(changed[:8]))
