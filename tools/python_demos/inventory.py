"""Read the unchanged Python demos without executing their simulations/imports."""

import argparse
import ast
from collections import Counter
import hashlib
import json
from pathlib import Path
import re
import sys


def module_leaf(name):
    return "core" if name in ("pychrono", "pychrono.core") else name.split(".", 1)[1]


def inspect_file(path, source_root):
    payload = path.read_bytes()
    tree = ast.parse(payload.decode("utf-8"), filename=str(path))
    imports, aliases, references, assets = set(), {}, set(), []
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            for alias in node.names:
                imports.add(alias.name)
                if alias.name == "pychrono" or alias.name.startswith("pychrono."):
                    aliases[alias.asname or alias.name.split(".")[0]] = module_leaf(alias.name)
        elif isinstance(node, ast.ImportFrom) and node.module:
            imports.add(node.module)
    for node in ast.walk(tree):
        if isinstance(node, ast.Attribute) and isinstance(node.value, ast.Name) and node.value.id in aliases:
            references.add((aliases[node.value.id], node.attr))
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute):
            if node.func.attr in ("GetChronoDataFile", "GetVehicleDataFile"):
                literal = node.args[0].value if node.args and isinstance(node.args[0], ast.Constant) else None
                assets.append({"line": node.lineno, "store": node.func.attr,
                               "literal": literal if isinstance(literal, str) else None,
                               "dynamic": not isinstance(literal, str)})
    helpers = set()
    for name in imports:
        candidate = path.parent / (name.split(".")[0] + ".py")
        if candidate.is_file() and candidate != path:
            helpers.add(candidate)
    # Two retained wheel-rig scripts explicitly load this parent-level helper
    # through importlib. Preserve that original relative-file convention.
    if any(isinstance(node, ast.Constant) and node.value == "SetChronoSolver.py" for node in ast.walk(tree)):
        candidate = path.parent.parent / "SetChronoSolver.py"
        if not candidate.is_file():
            raise ValueError("Retained solver helper is missing: " + str(candidate))
        helpers.add(candidate)
    modules = sorted({module_leaf(name) for name in imports if name == "pychrono" or name.startswith("pychrono.")})
    external = sorted({name.split(".")[0] for name in imports
                       if name.split(".")[0] not in sys.stdlib_module_names
                       and not name.startswith("pychrono")
                       and not (path.parent / (name.split(".")[0] + ".py")).is_file()})
    return {"path": path.relative_to(source_root).as_posix(), "bytes": len(payload),
            "sha256": hashlib.sha256(payload).hexdigest(), "imports": sorted(imports),
            "native_modules": modules, "external_packages": external,
            "helpers": sorted(helper.relative_to(source_root).as_posix() for helper in helpers),
            "potential_native_references": [{"module": module, "name": name} for module, name in sorted(references)],
            "literal_or_dynamic_asset_requests": assets}


def build_inventory(source_root):
    source_root = Path(source_root)
    python_root = source_root / "src/demos/python"
    files, programs = {}, []
    for source in sorted(python_root.rglob("demo_*.py")):
        record = inspect_file(source, source_root)
        closure, queue = {record["path"]}, list(record["helpers"])
        files[record["path"]] = record
        while queue:
            helper = queue.pop()
            if helper in closure:
                continue
            closure.add(helper)
            files[helper] = inspect_file(source_root / helper, source_root)
            queue.extend(files[helper]["helpers"])
        modules = sorted({value for path in closure for value in files[path]["native_modules"]})
        packages = sorted({value for path in closure for value in files[path]["external_packages"]})
        relative = source.relative_to(python_root)
        stem = re.sub(r"^demo_[A-Za-z0-9]+_", "", source.stem)
        name = re.sub(r"([a-z0-9])([A-Z])", r"\1_\2", stem).lower()
        required_profiles = []
        if "fsi" in modules:
            required_profiles += ["fsi-sph", "vsg"]
        if "sensor" in modules:
            required_profiles.append("sensor-optix")
        # The retained parser interface wraps both URDF and YAML unconditionally.
        if "parsers" in modules:
            required_profiles.append("yaml")
        if "ros" in modules and "sensor" in modules:
            required_profiles.append("ros-sensor")
        if "ros" in modules and "parsers" in modules:
            required_profiles.append("ros-urdf")
        programs.append({"name": name, "family": relative.parts[0], "source": record["path"],
                         "planned_target": "//examples/python/" + relative.parts[0] + ":" + name,
                         "native_modules": modules, "external_packages": packages,
                         "source_closure": sorted(closure), "required_profiles": required_profiles,
                         "numpy_bridge": "sensor" in modules or "numpy" in packages,
                         "status": "audited_not_yet_build_qualified", "runtime_qualified": False,
                         "asset_scope": "Literal requests are discovery evidence only; constructors, JSON, OBJ/MTL and imported CAD files add transitive assets."})
    if len(programs) != 97:
        raise ValueError("Expected the complete retained 97-program Python roster")
    return {"schema": "robodyna.python_demo_inventory.v1",
            "scope": "Source/import audit only. Native wrapper compilation, actual module import, and full demo runtime are separate gates.",
            "counts_by_family": dict(sorted(Counter(row["family"] for row in programs).items())),
            "programs": programs, "files": [files[key] for key in sorted(files)],
            "excluded_auxiliary_scope": "Unreferenced training/Blender utilities are not part of the original 97 demo_*.py mains.",
            "reference_scope": "Potential native references include conditional/fallback branches and are not asserted as unconditional API requirements."}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-root", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    output = build_inventory(args.source_root)
    with args.output.open("x") as stream:
        json.dump(output, stream, indent=2)
        stream.write("\n")
    print(json.dumps({"programs": len(output["programs"]), "source_files": len(output["files"]),
                      "families": output["counts_by_family"]}))


if __name__ == "__main__":
    main()
