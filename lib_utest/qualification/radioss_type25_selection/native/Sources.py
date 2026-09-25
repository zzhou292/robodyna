"""Pinned whole-routine extraction; no production numerical source participates."""
import hashlib
import json
from pathlib import Path
import re
ROOT = Path(__file__).resolve().parent


def read():
    result = {}
    for entry in json.loads((ROOT / "source-manifest.json").read_text())["files"]:
        path = ROOT.parent / entry["path"]
        data = path.read_bytes()
        if len(data) != entry["bytes"] or hashlib.sha256(data).hexdigest() != entry["sha256"]:
            raise RuntimeError("Pinned selection donor changed: " + entry["path"])
        result[path.name] = data.decode()
    return result


def routine(source, name):
    # Copy the complete original subroutine, including declarations and returns.
    start = "      SUBROUTINE " + name + "("
    assert source.count(start) == 1, name
    body = start + source.split(start, 1)[1]
    terminal = re.search(r"^      END[ \t]*$", body, re.MULTILINE)
    assert terminal, name
    return body[:terminal.end()] + "\n"


def constants(original, numerical_sources):
    # Resolve only native named constants used by retained whole routines. Keep
    # the original declaration/dependency order, not decimal reinterpretations.
    declarations = []
    for line in original.splitlines():
        match = re.search(r"my_real, parameter ::\s*(\w+)\s*=\s*(.*)", line)
        if match:
            declarations.append((match[1], match[2], line))
    names = {name for name, _, _ in declarations}
    requested = set(re.findall(r"\b[A-Z][A-Z_0-9]*\b", "\n".join(numerical_sources))) & names
    while True:
        expanded = requested.copy()
        for name, rhs, _ in declarations:
            if name in requested:
                expanded.update(set(re.findall(r"\b[A-Z][A-Z_0-9]*\b", rhs)) & names)
        if expanded == requested:
            break
        requested = expanded
    selected = [(name, line) for name, _, line in declarations if name in requested]
    if len({name for name, _ in selected}) != len(selected):
        raise RuntimeError("Ambiguous precision-dependent constant requires explicit resolution")
    return "module selection_constants\n use iso_c_binding\n implicit none\n" + \
        "\n".join(line.replace("my_real", "real(c_double)", 1) for _, line in selected) + \
        "\nend module\n"
