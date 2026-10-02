"""Inspect the real linked FMUs and their independently generated descriptors."""
import json
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET
import zipfile

from tests.fmi.elf.ExportsTest import inspect

manifest = json.loads(Path(sys.argv[1]).read_text())
arguments = sys.argv[2:]
if len(arguments) != 2 * len(manifest["exports"]):
    raise RuntimeError("Expected archive/library pairs for all six Vehicle FMUs")
for row, archive, library in zip(manifest["exports"], arguments[::2], arguments[1::2]):
    inspect(Path(library), "2")
    dynamic = subprocess.check_output(["/usr/bin/readelf", "-d", library], text=True)
    with zipfile.ZipFile(archive) as fmu:
        root = ET.fromstring(fmu.read("modelDescription.xml"))
        if root.get("guid") != row["guid"] or root.find(row["mode"]).get("modelIdentifier") != row["identifier"]:
            raise RuntimeError("Vehicle FMU identity/mode differs from original model")
        if not any(name.startswith("resources/") for name in fmu.namelist()):
            raise RuntimeError("Vehicle FMU lost its original runtime resources")
        runtime = "binaries/linux64/libIrrlicht.so.1.8"
        if (runtime in fmu.namelist()) != row["visualization"]:
            raise RuntimeError("Vehicle FMU visualization dependency is incomplete")
        if row["visualization"] and ("$ORIGIN" not in dynamic or "libIrrlicht.so.1.8" not in dynamic):
            raise RuntimeError("Vehicle FMU cannot resolve its packaged visual dependency")
print("All six native Vehicle FMUs preserve their C ABI, identity, resource and selected visual dependency boundaries")
