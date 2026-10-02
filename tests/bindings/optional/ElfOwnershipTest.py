"""Inspect actual optional DSOs for unique core/module implementation ownership."""
import json
from pathlib import Path
import subprocess
import sys
from python.runfiles import runfiles
from tests.bindings.elf_link_test import FORBIDDEN_DEFINITIONS, inspect

resolver = runfiles.Create()
paths = {}
for item in sys.argv[1:]:
    name, logical = item.split("=", 1)
    if name in paths:
        raise RuntimeError("Repeated ownership input: " + name)
    paths[name] = Path(resolver.Rlocation(logical)).resolve(strict=True)
records = {name: inspect(path) for name, path in paths.items()}
if records["core"]["soname"] != ["librobodyna_core.so"]:
    raise RuntimeError("Wrong core owner SONAME")
if set(records["core"]["core_definitions"]) != set(FORBIDDEN_DEFINITIONS):
    raise RuntimeError("Core lacks its retained mechanics ownership")
for name, record in records.items():
    if name != "core" and record["core_definitions"]:
        raise RuntimeError("Duplicated core implementation: " + name)

symbols = {name: subprocess.check_output(["/usr/bin/nm", "-D", "--defined-only", "--demangle", str(path)], text=True)
           for name, path in paths.items()}
expected_owners = {
    "images": " stbi_load\n",
    "vsg": "chrono::vsg3d::ChVisualSystemVSG::ChVisualSystemVSG(",
    "irrlicht": "chrono::irrlicht::ChVisualSystemIrrlicht::ChVisualSystemIrrlicht(",
    "postprocess": "chrono::postprocess::ChPovRay::ChPovRay(",
    "vehicle": "chrono::vehicle::FlatTerrain::FlatTerrain(",
}
for expected, symbol in expected_owners.items():
    owners = [name for name, table in symbols.items() if symbol in table]
    if owners != [expected]:
        raise RuntimeError("Unexpected definition owners for " + symbol + ": " + repr(owners))
    if expected != "images":
        wrapper = records[expected + "_wrapper"]
        soname = records[expected]["soname"]
        if len(soname) != 1 or wrapper["needed"].count(soname[0]) != 1:
            raise RuntimeError("Optional wrapper does not require its declared implementation: " + expected)
print(json.dumps({"schema": "robodyna.optional_binding_elf_ownership.v1", "status": "pass", "objects": records,
                  "scope": "Real loaded-link artifacts; unique selected out-of-line core/module/STB definitions, weak templates permitted"}, indent=2))
