"""Read-only checks for the source-preserving Bazel transition.

This checks source declarations, not Bazel graph evaluation or compilation.
Run from a checkout, or use ``bazel run`` which sets BUILD_WORKSPACE_DIRECTORY.
"""

import argparse
import ast
import hashlib
import json
import os
from pathlib import Path
import re

try:
    from build_defs.build_overlays import verify_build_overlay
except ModuleNotFoundError:
    from build_overlays import verify_build_overlay  # Direct checkout script invocation.


def workspace_path(root, relative):
    """Resolve an owned path without allowing absolute paths or symlink escape."""
    name = Path(relative)
    if name.is_absolute() or ".." in name.parts:
        raise ValueError(f"Expected a repository-relative path: {relative}")
    result = (root / name).resolve()
    if not result.is_relative_to(root.resolve()):
        raise ValueError(f"Source path escapes the checkout: {relative}")
    return result


def declarations(path):
    """Read literal call attributes without evaluating Starlark or loading rules."""
    result = []
    for node in ast.parse(path.read_text(), filename=str(path)).body:
        if not isinstance(node, ast.Expr) or not isinstance(node.value, ast.Call):
            continue
        call = node.value
        if not isinstance(call.func, ast.Name):
            continue
        attributes = {}
        for keyword in call.keywords:
            try:
                attributes[keyword.arg] = ast.literal_eval(keyword.value)
            except (ValueError, TypeError):
                pass  # A computed attribute is not a proved literal declaration.
        result.append((call.func.id, attributes))
    return result


def inspect_workspace(root, contract):
    """Return explicit violations; do not mutate the workspace or start tools."""
    root = Path(root).resolve()
    errors = []

    def check(condition, message):
        if not condition:
            errors.append(message)

    try:
        source = workspace_path(root, contract["source_root"])
        overlays = {}
        overlay_path = root / "build_defs/legacy/build_overlays.json"
        if overlay_path.is_file():
            overlay_document = json.loads(overlay_path.read_text())
            check(overlay_document["schema_version"] == 1 and
                  overlay_document["qualified_commit"] == contract["qualified_commit"],
                  "Build metadata overlay belongs to a different qualified source")
            for relative, entry in overlay_document["files"].items():
                current = workspace_path(source, relative).read_bytes()
                overlays[relative] = verify_build_overlay(relative, current, entry)
        module = declarations(root / "MODULE.bazel")
        check(any(kind == "module" and args.get("name") == "robodyna"
                  for kind, args in module), "Root module must be robodyna")
        check(any(kind == "local_repository" and args.get("name") == contract["repository_name"]
                  and args.get("path") == contract["source_root"] for kind, args in module),
              "Legacy FEA must map to the owned source directory")
        for kind, args in module:
            if kind in ("local_repository", "new_local_repository") and "path" in args:
                workspace_path(root, args["path"])
        versions = {args.get("name"): args.get("version")
                    for kind, args in module if kind == "bazel_dep"}
        for name, version in contract["dependency_versions"].items():
            check(versions.get(name) == version, f"Dependency version drift: {name}")
        overrides = {args.get("module_name"): args for kind, args in module
                     if kind == "single_version_override"}
        for name in ("rules_cuda", "rules_foreign_cc", "googletest"):
            check(overrides.get(name, {}).get("patch_strip") == 1,
                  f"Qualified compatibility patch missing: {name}")
            expected_patches = ["//build_defs/patches:" + patch
                                for patch in contract["patches"] if patch.startswith(name + "_")]
            if expected_patches:
                check(overrides.get(name, {}).get("patches") == expected_patches,
                      f"Qualified compatibility patch not applied: {name}")
        for relative, expected in contract["source_files"].items():
            path = workspace_path(source, relative)
            check(path.is_file() and (hashlib.sha256(path.read_bytes()).hexdigest() == expected or
                                      overlays.get(relative) == expected),
                  f"Qualified source changed or missing: {relative}")
        for name, expected in contract["patches"].items():
            path = workspace_path(root, "build_defs/patches/" + name)
            check(path.is_file() and hashlib.sha256(path.read_bytes()).hexdigest() == expected,
                  f"Qualified patch changed or missing: {name}")
        check((root / ".bazelversion").read_text().strip() == "9.2.0", "Bazel version drift")
        options = [line.strip() for line in (root / ".bazelrc").read_text().splitlines()
                   if line.strip() and not line.lstrip().startswith("#")]
        check(not any(flag in line for line in options for flag in
                      ("--use_fast_math", "-ffast-math", "--fmad", "--ftz", "-ffp-contract")),
              "Numerical policies must remain target-local, not global .bazelrc flags")
        ignored = {line.strip() for line in (root / ".bazelignore").read_text().splitlines()}
        check(contract["source_root"] in ignored,
              "Imported FEA must not be loaded twice through main and legacy repository labels")
        facade = declarations(root / "src/fea/BUILD.bazel")
        for kind, args in facade:
            if kind != "alias":
                continue
            label = args.get("actual", "")
            match = re.fullmatch(r"@legacy_fea//([^:]+):([^:]+)", label)
            check(match is not None, f"Unreviewed FEA facade target: {label}")
            if not match:
                continue
            package = workspace_path(source, match[1])
            build = package / "BUILD.bazel"
            if not build.is_file():
                build = package / "BUILD"
            check(build.is_file(), f"Missing source package for {label}")
            if build.is_file():
                names = {values.get("name") for _, values in declarations(build)}
                check(match[2] in names, f"Missing inherited target: {label}")
    except (OSError, ValueError, KeyError, SyntaxError) as error:
        errors.append(str(error))
    return errors


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", type=Path,
                        default=Path(os.environ.get("BUILD_WORKSPACE_DIRECTORY", Path.cwd())))
    args = parser.parse_args()
    root = args.workspace.resolve()
    contract = json.loads((root / "build_defs/legacy/source_contract.json").read_text())
    errors = inspect_workspace(root, contract)
    print(json.dumps({"passed": not errors, "scope": "static source boundary only",
                      "source_commit": contract["qualified_commit"], "errors": errors}, indent=2))
    return bool(errors)


if __name__ == "__main__":
    raise SystemExit(main())
