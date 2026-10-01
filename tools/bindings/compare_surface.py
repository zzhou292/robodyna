"""Compare complete generated binding surfaces against genuine historical inputs."""

import argparse
import ast
import hashlib
import json
from pathlib import Path


class WithoutDocstrings(ast.NodeTransformer):
    """C++ spelling in generated documentation is not callable Python behavior."""
    def strip(self, node):
        self.generic_visit(node)
        if (node.body and isinstance(node.body[0], ast.Expr)
                and isinstance(node.body[0].value, ast.Constant)
                and isinstance(node.body[0].value.value, str)):
            node.body = node.body[1:]
        return node
    visit_ClassDef = strip
    visit_FunctionDef = strip
    visit_AsyncFunctionDef = strip


def python_surface(path):
    tree = WithoutDocstrings().visit(ast.parse(path.read_text()))
    result = {}
    for node in tree.body:
        if isinstance(node, (ast.ClassDef, ast.FunctionDef)) and not node.name.startswith("_"):
            result[node.name] = ast.dump(node, include_attributes=False)
        elif isinstance(node, (ast.Assign, ast.AnnAssign)):
            targets = node.targets if isinstance(node, ast.Assign) else [node.target]
            for target in targets:
                if isinstance(target, ast.Name) and not target.id.startswith("_"):
                    result[target.id] = ast.dump(node, include_attributes=False)
    return result


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def difference(old, new):
    return {"removed": sorted(set(old) - set(new)), "added": sorted(set(new) - set(old)),
            "changed": sorted(key for key in set(old) & set(new) if old[key] != new[key])}


def diagnostic_text(text, source_root):
    """Canonicalize only the explicitly declared snapshot root, not warnings."""
    if source_root is None:
        return text
    prefix = str(Path(source_root).resolve()) + "/"
    return "".join("<parser-inputs>/" + line[len(prefix):] if line.startswith(prefix) else line
                   for line in text.splitlines(keepends=True))


def compare(baseline, candidate, module="core", languages=("python", "csharp"),
            baseline_inputs=None, candidate_inputs=None):
    if module not in ("core", "fea", "vehicle") or not languages or not set(languages) <= {"python", "csharp"}:
        raise ValueError("Select a declared module and at least one language")
    if module == "fea" and "csharp" in languages:
        raise ValueError("The retained sources do not provide a separate C# FEA module")
    if (baseline_inputs is None) != (candidate_inputs is None):
        raise ValueError("Both diagnostic input roots must be declared together")
    result = {"schema": "robodyna.swig_surface_comparison.v1", "module": module,
              "languages": list(languages), "passed": True,
              "scope": "Entire public Python AST excluding docstrings, exact C# files, generation diagnostics; not compilation/runtime"}
    if "python" in languages:
        old = python_surface(baseline / "python" / (module + ".py"))
        new = python_surface(candidate / "python" / (module + ".py"))
        if not old or not new:
            raise ValueError("A selected Python module has no public generated surface")
        delta = difference(old, new)
        result.update(python_baseline_count=len(old), python_candidate_count=len(new), python=delta)
        result["passed"] = result["passed"] and not any(delta.values())
    if "csharp" in languages:
        old = {p.name: digest(p) for p in (baseline / "csharp").glob("*.cs")}
        new = {p.name: digest(p) for p in (candidate / "csharp").glob("*.cs")}
        if not old or not new:
            raise ValueError("A selected C# module has no generated proxy files")
        delta = difference(old, new)
        result.update(csharp_baseline_files=len(old), csharp_candidate_files=len(new), csharp=delta)
        result["passed"] = result["passed"] and not any(delta.values())
    warnings = {}
    for profile, directory in (("baseline", baseline), ("candidate", candidate)):
        warnings[profile] = {language: (directory / language / "stderr.log").read_text() for language in languages}
    result["diagnostics"] = warnings
    normalized = {
        profile: {language: diagnostic_text(warnings[profile][language], root) for language in languages}
        for profile, root in (("baseline", baseline_inputs), ("candidate", candidate_inputs))
    }
    result["compared_diagnostics"] = normalized
    result["diagnostic_root_mapping"] = {
        "baseline": str(baseline_inputs) if baseline_inputs else None,
        "candidate": str(candidate_inputs) if candidate_inputs else None,
        "scope": "Only exact leading snapshot roots; preserve relative paths, line numbers, codes and messages",
    }
    result["passed"] = result["passed"] and normalized["baseline"] == normalized["candidate"]
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline", type=Path, required=True)
    parser.add_argument("--candidate", type=Path, required=True)
    parser.add_argument("--report", type=Path, required=True)
    parser.add_argument("--module", choices=("core", "fea", "vehicle"), default="core")
    parser.add_argument("--languages", nargs="+", choices=("python", "csharp"), default=["python", "csharp"])
    parser.add_argument("--baseline-inputs", type=Path)
    parser.add_argument("--candidate-inputs", type=Path)
    args = parser.parse_args()
    result = compare(args.baseline, args.candidate, args.module, args.languages,
                     args.baseline_inputs, args.candidate_inputs)
    with args.report.open("x") as stream:
        json.dump(result, stream, indent=2, sort_keys=True)
        stream.write("\n")
    print(json.dumps({key: value for key, value in result.items() if key not in ("scope", "diagnostics")}))
    raise SystemExit(0 if result["passed"] else 1)
