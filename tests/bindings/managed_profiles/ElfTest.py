"""Require one actual implementation owner behind each managed profile wrapper."""
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
        raise RuntimeError("Repeated native owner name")
    paths[name] = Path(resolver.Rlocation(logical)).resolve(strict=True)
records = {name: inspect(path) for name, path in paths.items()}
if records["core"]["soname"] != ["librobodyna_core.so"] or set(records["core"]["core_definitions"]) != set(FORBIDDEN_DEFINITIONS):
    raise RuntimeError("Wrong shared native core identity")
for name, record in records.items():
    if name != "core" and record["core_definitions"]:
        raise RuntimeError("Duplicated out-of-line core implementation: " + name)
    if name.endswith("_wrapper"):
        owner = name.removesuffix("_wrapper")
        sonames = records[owner]["soname"]
        if len(sonames) != 1 or record["needed"].count(sonames[0]) != 1:
            raise RuntimeError("Wrapper does not require its actual native owner: " + name)
symbols = {name: subprocess.check_output(["/usr/bin/nm", "-D", "--defined-only", "--demangle", str(path)], text=True)
           for name, path in paths.items()}
for owner, symbol in {"vehicle": "chrono::vehicle::CRGTerrain::CRGTerrain(",
                      "sensor": "chrono::sensor::ChGPSSensor::ChGPSSensor(",
                      "ros": "chrono::ros::ChROSGPSHandler::ChROSGPSHandler("}.items():
    if owner in paths:
        found = [name for name, text in symbols.items() if symbol in text]
        if found != [owner]:
            raise RuntimeError("Wrong optional implementation owner: " + symbol + ": " + repr(found))
print(json.dumps({"schema": "robodyna.managed_profile_elf.v1", "passed": True, "objects": records}, indent=2))
