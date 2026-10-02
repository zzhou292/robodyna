"""Audit actual generated PythonOCC imports without executing CAD or Python code."""

import ast
from pathlib import Path


def module_imports(path, available_modules):
    """Return native proxy dependencies, original helpers and other imports."""
    modules, helpers, external = set(), set(), set()
    for node in ast.walk(ast.parse(Path(path).read_text())):
        names = []
        if isinstance(node, ast.Import):
            names = [item.name for item in node.names]
        elif isinstance(node, ast.ImportFrom):
            if node.module in ("OCC.Core", "OCC.Wrapper"):
                names = [node.module + "." + item.name for item in node.names]
            elif node.module:
                names = [node.module]
            else:
                names = [item.name for item in node.names]
        for name in names:
            leaf = name.removeprefix("OCC.Core.")
            if leaf in available_modules:
                modules.add(leaf)
            elif name.startswith("OCC.") and name not in ("OCC.Core",):
                helpers.add(name)
            elif name != "OCC":
                external.add(name)
    return {"modules": sorted(modules), "helpers": sorted(helpers), "external": sorted(external)}


def complete_closure(directory, entry_modules):
    """Discover the closed native roster, including relative proxy imports."""
    directory = Path(directory)
    available = {path.stem for path in directory.glob("*.py")}
    pending = list(entry_modules)
    rows = {}
    while pending:
        name = pending.pop()
        if name in rows:
            continue
        if name not in available:
            raise ValueError("Required generated PythonOCC proxy is absent: " + name)
        row = module_imports(directory / (name + ".py"), available)
        rows[name] = row
        pending.extend(set(row["modules"]) - set(rows))
    return {name: rows[name] for name in sorted(rows)}


def verify_selection(directory, selection):
    """Check real newly-generated import edges against the reviewed manifest."""
    actual = complete_closure(directory, selection["entry_modules"])
    if sorted(actual) != selection["modules"]:
        raise ValueError("Generated PythonOCC dependency closure differs from selected modules")
    if actual != selection["generated_imports"]:
        raise ValueError("Generated PythonOCC import/helper/native-extension edges changed")
    return actual
