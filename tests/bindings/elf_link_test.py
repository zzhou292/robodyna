"""Verify wrappers share the native core instead of defining another copy."""

import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys
from python.runfiles import runfiles


FORBIDDEN_DEFINITIONS = (
    "robodyna::mbd::RbBody::RbBody(",
    "robodyna::mbd::RbBody::~RbBody(",
    "robodyna::mbd::RbBody::Update(",
    "chrono::ChClassFactory::GetGlobalClassFactory(",
    "robodyna::fea::RbMesh::AddNode(",
    "robodyna::fea::RbMesh::SetupInitial(",
    "chrono::fea::ChElementSpring::SetNodes(",
)


def inspect(path):
    dynamic = subprocess.check_output(["/usr/bin/readelf", "-d", str(path)], text=True)
    defined = subprocess.check_output(["/usr/bin/nm", "-D", "--defined-only", "--demangle", str(path)], text=True)
    undefined = subprocess.check_output(["/usr/bin/nm", "-D", "--undefined-only", "--demangle", str(path)], text=True)
    return {
        "path": str(path), "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
        "needed": re.findall(r"\(NEEDED\).*\[([^]]+)\]", dynamic),
        "soname": re.findall(r"\(SONAME\).*\[([^]]+)\]", dynamic),
        "core_definitions": [name for name in FORBIDDEN_DEFINITIONS if name in defined],
        "core_references": [name for name in FORBIDDEN_DEFINITIONS if name in undefined],
    }


if __name__ == "__main__":
    if len(sys.argv) != 5:
        raise RuntimeError("Expected backend, Python core, C# core and Python FEA wrapper paths")
    resolver = runfiles.Create()
    paths = [Path(resolver.Rlocation(name)).resolve(strict=True) for name in sys.argv[1:]]
    backend, python, csharp, fea = [inspect(path) for path in paths]
    if backend["soname"] != ["librobodyna_core.so"] or set(backend["core_definitions"]) != set(FORBIDDEN_DEFINITIONS):
        raise RuntimeError("Declared native backend lacks its expected SONAME or owning core definitions")
    for wrapper in (python, csharp, fea):
        if wrapper["needed"].count("librobodyna_core.so") != 1:
            raise RuntimeError("Wrapper does not require the one declared shared core: " + wrapper["path"])
        if wrapper["core_definitions"]:
            raise RuntimeError("Wrapper embeds out-of-line core implementation symbols: " + repr(wrapper))
        required = "robodyna::fea::RbMesh::AddNode(" if wrapper is fea else "robodyna::mbd::RbBody::RbBody("
        if required not in wrapper["core_references"]:
            raise RuntimeError("Wrapper does not resolve its expected native operation externally: " + required)
    tools = {name: hashlib.sha256(Path(name).read_bytes()).hexdigest()
             for name in ("/usr/bin/readelf", "/usr/bin/nm")}
    print(json.dumps({"schema": "robodyna.binding_elf_ownership.v1", "passed": True,
                      "backend": backend, "wrappers": [python, csharp, fea], "tools": tools,
                      "scope": "Out-of-line body/mesh/FE/factory ownership; normal weak inline/template symbols are permitted"}, indent=2))
