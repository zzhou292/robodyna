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
    if len(sys.argv) != 4:
        raise RuntimeError("Expected backend, Python wrapper and C# wrapper runfile paths")
    resolver = runfiles.Create()
    paths = [Path(resolver.Rlocation(name)).resolve(strict=True) for name in sys.argv[1:]]
    backend, python, csharp = [inspect(path) for path in paths]
    if backend["soname"] != ["librobodyna_core.so"] or set(backend["core_definitions"]) != set(FORBIDDEN_DEFINITIONS):
        raise RuntimeError("Declared native backend lacks its expected SONAME or owning core definitions")
    for wrapper in (python, csharp):
        if wrapper["needed"].count("librobodyna_core.so") != 1:
            raise RuntimeError("Wrapper does not require the one declared shared core: " + wrapper["path"])
        if wrapper["core_definitions"]:
            raise RuntimeError("Wrapper embeds out-of-line core implementation symbols: " + repr(wrapper))
        if "robodyna::mbd::RbBody::RbBody(" not in wrapper["core_references"]:
            raise RuntimeError("Wrapper does not resolve body construction externally")
    tools = {name: hashlib.sha256(Path(name).read_bytes()).hexdigest()
             for name in ("/usr/bin/readelf", "/usr/bin/nm")}
    print(json.dumps({"schema": "robodyna.binding_elf_ownership.v1", "passed": True,
                      "backend": backend, "wrappers": [python, csharp], "tools": tools,
                      "scope": "Out-of-line body/factory ownership; normal weak inline/template symbols are permitted"}, indent=2))
