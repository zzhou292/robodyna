"""Require the public FMU C ABI and keep native C++ state private to each FMU."""

import json
from pathlib import Path
import subprocess
import sys


def inspect(path, version):
    output = subprocess.check_output(["/usr/bin/nm", "-D", "--defined-only", "--format=posix", str(path)], text=True)
    names = {line.split()[0] for line in output.splitlines() if line.strip()}
    prefix = "fmi" + version
    unexpected = sorted(name for name in names if name != "createModelDescription" and not name.startswith(prefix))
    if unexpected:
        raise RuntimeError("FMU exposes non-interface symbols: " + str(path) + ": " + repr(unexpected[:12]))
    required = {"createModelDescription", prefix + "GetVersion", prefix + "DoStep"}
    if not required.issubset(names):
        raise RuntimeError("FMU lacks its expected public interface: " + str(path))
    return {"library": str(path), "version": version, "public_symbols": sorted(names)}


if __name__ == "__main__":
    if len(sys.argv) != 17:
        raise RuntimeError("Expected eight FMU library/version pairs")
    results = [inspect(Path(path), version) for path, version in zip(sys.argv[1::2], sys.argv[2::2])]
    print(json.dumps({"schema": "robodyna.fmu_elf_scope.v1", "passed": True, "libraries": results}, indent=2))
