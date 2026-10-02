"""Compose generated proxies in the retained CMake module order, with an audit."""

import argparse
import hashlib
import json
from pathlib import Path
import shutil


def compose(modules, output, approvals=()):
    approved = {tuple(row[key] for key in ("file", "kept_module", "discarded_module",
                                          "kept_sha256", "discarded_sha256")) for row in approvals}
    if len(approved) != len(approvals):
        raise ValueError("Duplicate managed collision approval")
    used = set()
    selected = {}
    duplicates = []
    order = []
    for module, directory in modules:
        if module in order or not module or not directory.is_dir():
            raise ValueError(f"Invalid or repeated managed module: {module}")
        order.append(module)
        files = sorted(directory.glob("*.cs"))
        if not files:
            raise ValueError(f"No generated C# sources in module {module}")
        for source in files:
            digest = hashlib.sha256(source.read_bytes()).hexdigest()
            current = {"module": module, "path": str(source), "sha256": digest}
            if source.name not in selected:
                selected[source.name] = current
                continue
            previous = selected[source.name]
            decision = {"file": source.name, "kept_module": previous["module"], "discarded_module": module,
                        "kept_sha256": previous["sha256"], "discarded_sha256": digest}
            key = tuple(decision.values())
            if previous["sha256"] != digest:
                if key not in approved:
                    raise ValueError("Differing generated proxy requires explicit review: " + json.dumps(decision))
                used.add(key)
            decision["identical"] = previous["sha256"] == digest
            duplicates.append(decision)
    if used != approved:
        raise ValueError("Stale or unused managed collision approval")
    # Bazel may create the declared tree-artifact directory before invoking the
    # action. Admit that empty directory, but never overwrite prior contents or
    # follow a directory symlink supplied as the output.
    if output.is_symlink():
        raise ValueError("Managed proxy output must not be a symlink")
    if output.exists():
        if not output.is_dir() or any(output.iterdir()):
            raise ValueError("Managed proxy output must be an empty directory")
    else:
        output.mkdir(parents=True)
    for name, record in selected.items():
        shutil.copyfile(record["path"], output / name)
    return {"schema": "robodyna.managed_proxy_composition.v1", "module_order": order,
            "selected": selected, "duplicates": duplicates,
            "policy": "Retained first-module ownership; differing proxy bodies require exact reviewed hash pairs"}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--module", action="append", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--receipt", type=Path, required=True)
    parser.add_argument("--approvals", type=Path)
    args = parser.parse_args()
    modules = [(name, Path(path)) for name, path in (value.split("=", 1) for value in args.module)]
    approvals = json.loads(args.approvals.read_text())["collisions"] if args.approvals else []
    result = compose(modules, args.output, approvals)
    args.receipt.write_text(json.dumps(result, indent=2) + "\n")


if __name__ == "__main__":
    main()
