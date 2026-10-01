"""Read the explicit owned parser-view registry; never discover views by glob."""

import ast
import hashlib
import json
from pathlib import Path
import re


REGISTRY = "build_defs/bindings/declaration_views.json"


def relative_path(value):
    if not isinstance(value, str) or not value or Path(value).is_absolute() or ".." in Path(value).parts:
        raise ValueError("Declaration registry paths must stay repository-relative")
    return value


def read(root):
    root = Path(root)
    path = root / REGISTRY
    if path.stat().st_size > 64 * 1024:
        raise ValueError("Declaration registry exceeds its 64 KiB metadata bound")
    document = json.loads(path.read_text())
    if document.get("schema") != "robodyna.swig_declaration_views.v1" or not document.get("views"):
        raise ValueError("Invalid or empty declaration registry")
    result, names, outputs = [], set(), set()
    required = {"name", "contract", "ledger", "canonical", "forwarder", "requires_fea"}
    for entry in document["views"]:
        if set(entry) != required or not re.fullmatch(r"[a-z][a-z0-9_]*", entry["name"]):
            raise ValueError("Malformed declared view entry")
        if entry["name"] in names or type(entry["requires_fea"]) is not bool:
            raise ValueError("Duplicate view or invalid FEA condition")
        names.add(entry["name"])
        for key in ("contract", "ledger", "canonical", "forwarder"):
            relative_path(entry[key])
        contract = json.loads((root / entry["contract"]).read_text())
        output = contract.get("output", "")
        if not re.fullmatch(r"robodyna_swig/[A-Za-z0-9_]+[.]h", output) or output in outputs:
            raise ValueError("Invalid or duplicate declaration output")
        if contract.get("original_path") != entry["forwarder"]:
            raise ValueError("Registry forwarder differs from the authenticated original header")
        ledger_bytes = (root / entry["ledger"]).read_bytes()
        if hashlib.sha256(ledger_bytes).hexdigest() != contract.get("expected_ledger_sha256"):
            raise ValueError("Declaration registry ledger differs from its reviewed pin")
        ledger = json.loads(ledger_bytes)
        selected = [row for row in ledger.get("files", []) if row.get("original_path") == entry["forwarder"]]
        if ledger.get("schema") != "robodyna.source_transformations.v1" or len(selected) != 1:
            raise ValueError("Declaration registry must select exactly one original header")
        if selected[0].get("canonical_path") != entry["canonical"]:
            raise ValueError("Registry canonical dependency differs from the source ledger")
        if selected[0].get("original_sha256") != contract.get("expected_original_sha256"):
            raise ValueError("Registry immutable header identity differs from the contract")
        outputs.add(output)
        result.append(dict(entry, output=output))
    return result


def enabled(root, fea_enabled):
    return [entry for entry in read(root) if fea_enabled or not entry["requires_fea"]]


def verify_bazel(root):
    """Check actual declared actions/default inputs against the owned registry."""
    root = Path(root)
    specs = read(root)
    build = ast.parse((root / "build_defs/bindings/BUILD.bazel").read_text())
    actions = {}
    for node in build.body:
        if isinstance(node, ast.Expr) and isinstance(node.value, ast.Call) and isinstance(node.value.func, ast.Name):
            if node.value.func.id == "declaration_view":
                values = {kw.arg: ast.literal_eval(kw.value) for kw in node.value.keywords}
                actions[values["name"]] = values
    if set(actions) != {entry["name"] + "_declarations" for entry in specs}:
        raise ValueError("Bazel declaration action set differs from the registry")
    for entry in specs:
        action = actions[entry["name"] + "_declarations"]
        contract = json.loads((root / entry["contract"]).read_text())
        expected = {"canonical_path": entry["canonical"], "original_path": entry["forwarder"],
                    "view_name": Path(entry["output"]).name,
                    "original_sha256": contract["expected_original_sha256"],
                    "ledger_sha256": contract["expected_ledger_sha256"]}
        if any(action.get(key) != value for key, value in expected.items()):
            raise ValueError("Bazel declaration pins/paths differ: " + entry["name"])
        for key, field in (("canonical", "canonical"), ("forwarder", "forwarder"), ("ledger", "ledger")):
            if action[key].removeprefix("//").replace(":", "/") != entry[field]:
                raise ValueError("Bazel declaration input differs: " + entry["name"])
    defaults = []
    for node in ast.walk(ast.parse((root / "build_defs/bindings/swig.bzl").read_text())):
        if isinstance(node, ast.Dict):
            for key, value in zip(node.keys, node.values):
                if isinstance(key, ast.Constant) and key.value == "declarations" and isinstance(value, ast.Call):
                    defaults += [ast.literal_eval(kw.value) for kw in value.keywords if kw.arg == "default"]
    expected = {"//build_defs/bindings:" + entry["name"] + "_header" for entry in specs}
    if len(defaults) != 1 or set(defaults[0]) != expected or len(defaults[0]) != len(expected):
        raise ValueError("SWIG default declaration inputs differ from the registry")
    return len(specs)
