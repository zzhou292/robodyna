"""Validate the declared native app port without running Bazel or compilers."""

import argparse
import hashlib
import json
import os
from pathlib import Path

from build_defs.app.cmake_sources import library_source
from build_defs.source_boundary import declarations, workspace_path


def check_port(root, manifest):
    root = Path(root).resolve()
    errors = []
    seen_sources = set()
    targets = manifest["targets"]
    mapping = dict(manifest["external_targets"])
    mapping.update({name: record["bazel_target"] for name, record in targets.items()})

    def require(condition, message):
        if not condition:
            errors.append(message)

    try:
        app = workspace_path(root, manifest["source_root"])
        for name, record in targets.items():
            cmake = workspace_path(app, record["cmake_file"])
            require(hashlib.sha256(cmake.read_bytes()).hexdigest() == record["cmake_sha256"],
                    f"Unreviewed CMake source change: {record['cmake_file']}")
            sources, dependencies, copts = library_source(cmake.read_text(), name, record["cmake_file"])
            require(sources == record["sources"], f"Source list differs from CMake: {name}")
            require(dependencies == record["dependencies"], f"Dependency list differs from CMake: {name}")
            require(copts == record["copts"], f"Numeric options differ from CMake: {name}")
            require(record["bazel_dependencies"] == [mapping[item] for item in dependencies],
                    f"Dependency mapping differs: {name}")
            package, target = record["bazel_target"].removeprefix("//").split(":")
            build = workspace_path(root, package + "/BUILD.bazel")
            rules = [args for kind, args in declarations(build)
                     if kind == "app_library" and args.get("name") == target]
            require(len(rules) == 1, f"Missing or duplicate native library declaration: {name}")
            if len(rules) == 1:
                for attr, key in (("srcs", "sources"), ("hdrs", "headers"),
                                  ("deps", "bazel_dependencies"), ("copts", "copts")):
                    require(rules[0].get(attr) == record[key], f"Bazel {attr} differs from manifest: {name}")
                require(rules[0].get("header_deps", []) == record.get("header_dependencies", []),
                        f"Explicit header dependencies differ from manifest: {name}")
                evidence = record.get("header_dependency_evidence", [])
                require([item["target"] for item in evidence] == record.get("header_dependencies", []),
                        f"Header dependency lacks owning-source evidence: {name}")
            for relative in record["sources"] + record["headers"]:
                require(workspace_path(app, relative).is_file(), f"Missing declared source input: {relative}")
                require(not {"tests", "frozen"}.intersection(Path(relative).parts),
                        f"Test/frozen source in production: {relative}")
            for relative in record["sources"]:
                require(relative not in seen_sources, f"Translation unit compiled by two app targets: {relative}")
                seen_sources.add(relative)
        visited, pending = set(), set()

        def visit(name):
            if name in visited or name not in targets:
                return
            if name in pending:
                raise ValueError(f"Application library dependency cycle at {name}")
            pending.add(name)
            for dependency in targets[name]["dependencies"]:
                visit(dependency)
            pending.remove(name)
            visited.add(name)

        for name in targets:
            visit(name)
        require(not any("gtest" in label.lower() or "fortran" in label.lower()
                        for label in manifest["external_targets"].values()),
                "Production app must not depend on GTest or a Fortran oracle")
    except (OSError, ValueError, KeyError, SyntaxError) as error:
        errors.append(str(error))
    return errors


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", type=Path,
                        default=Path(os.environ.get("BUILD_WORKSPACE_DIRECTORY", Path.cwd())))
    args = parser.parse_args()
    root = args.workspace.resolve()
    manifest = json.loads((root / "build_defs/app/production_targets.json").read_text())
    errors = check_port(root, manifest)
    print(json.dumps({"passed": not errors, "scope": "static native app source port only",
                      "libraries": len(manifest["targets"]),
                      "translation_units": sum(len(record["sources"]) for record in manifest["targets"].values()),
                      "errors": errors}, indent=2))
    return bool(errors)


if __name__ == "__main__":
    raise SystemExit(main())
