"""Shared actual-import/owner checks for explicitly configured binding packages."""

import argparse
import importlib
import json
from pathlib import Path
import runpy
import sys
import tempfile

sys.path.insert(0, str(Path(__file__).parent.parent))
from origin import require_declared_origin


def require(value, message):
    if not value:
        raise RuntimeError(message)


def open_package():
    parser = argparse.ArgumentParser()
    parser.add_argument("--package-manifest", type=Path, required=True)
    args = parser.parse_args()
    manifest = args.package_manifest.absolute()
    document = json.loads(manifest.read_text())
    sys.path.insert(0, str(manifest.parent))
    product = importlib.import_module("robodyna")
    if document["numpy"]:
        require_declared_origin(importlib.import_module("numpy"), manifest, document, "numpy/__init__.py")
    for item in document["modules"]:
        name = item["name"]
        require(importlib.import_module("robodyna." + name) is importlib.import_module("pychrono." + name),
                "Duplicate public/compatibility native module: " + name)
    return product, manifest, document


def finish(manifest, document, checks, steps=0):
    owners = {}
    for line in Path("/proc/self/maps").read_text().splitlines():
        fields = line.split(maxsplit=5)
        if len(fields) != 6:
            continue
        require("libChrono_" not in fields[5], "An old separately installed Chrono library was loaded")
        basename = Path(fields[5]).name
        if basename.startswith("librobodyna_"):
            owners.setdefault(basename, set()).add((fields[3], fields[4]))
    for backend in document["backends"]:
        name = Path(backend["path"]).name
        require(len(owners.get(name, set())) == 1, "Missing or duplicated native owner: " + name)
    # Use a real second initializer path. Admission must fail before loading any
    # second native DSO; catching an unrelated missing-file error is not enough.
    with tempfile.TemporaryDirectory() as temporary:
        other = Path(temporary)
        (other / "pychrono").mkdir()
        (other / "package.json").write_text(json.dumps(document))
        init = other / "pychrono/__init__.py"
        init.write_bytes((manifest.parent / "pychrono/__init__.py").read_bytes())
        try:
            runpy.run_path(str(init), run_name="_robodyna_rejected_package")
        except ImportError as error:
            require("cannot mix binding packages" in str(error), "Second package failed for an unrelated reason")
        else:
            raise RuntimeError("Incompatible second package was admitted")
    print(json.dumps({"schema": "robodyna.python_profile_runtime.v1", "passed": True,
                      "profile": document["profile"], "abi_capabilities": document["abi_capabilities"],
                      "checks": checks + ["one native owner per declared DSO", "second package rejected before native loading"],
                      "physics_steps": steps, "render_or_gpu_execution": False}, indent=2))
