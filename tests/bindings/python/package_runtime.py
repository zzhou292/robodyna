"""Exercise the real public/compatibility imports and one shared native owner."""

import argparse
import gc
import importlib
import json
from pathlib import Path
import sys


def require(condition, description):
    if not condition:
        raise RuntimeError(description)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--package-manifest", type=Path, required=True)
    parser.add_argument("--with-fea", action="store_true")
    args = parser.parse_args()
    manifest = args.package_manifest.absolute()
    root = manifest.parent
    sys.path.insert(0, str(root))
    public = importlib.import_module("robodyna")
    old = importlib.import_module("pychrono")
    core = importlib.import_module("robodyna.core")
    require(core is importlib.import_module("pychrono.core"), "Core module loaded twice under different names")
    require(public.ChBody is old.ChBody is core.ChBody, "Facade created a different native body class")
    body = public.ChBody()
    body.SetMass(7.25)
    system = old.ChSystemSMC()
    system.SetNumThreads(1, 1, 1)
    system.AddBody(body)
    body_alias = system.GetBodies()[0]
    body_alias.SetMass(4.5)
    require(body.GetMass() == 4.5, "Public/compatibility paths do not share the actual body")
    checks = ["canonical and compatibility module identity", "shared native body instance"]
    if args.with_fea:
        fea = importlib.import_module("robodyna.fea")
        require(fea is importlib.import_module("pychrono.fea"), "FEA module was duplicated")
        mesh = fea.ChMesh()
        node = fea.ChNodeFEAxyz(core.ChVector3d(0, 0, 0))
        node.SetMass(2)
        mesh.AddNode(node)
        system.AddMesh(mesh)
        alias = fea.CastToChMesh(system.GetMeshes()[0])
        require(alias.GetNumNodes() == 1, "Core/FEA module ownership crossing failed")
        system.RemoveMesh(alias)
        del mesh, alias
        gc.collect()
        require(node.GetMass() == 2, "Detached shared FE node was not retained")
        checks.append("real cross-module mesh/node ownership")
    system.Clear()
    gc.collect()
    require(body_alias.GetMass() == 4.5, "Detached public body alias lost shared ownership")
    mapped = set()
    for line in Path("/proc/self/maps").read_text().splitlines():
        fields = line.split(maxsplit=5)
        if len(fields) != 6:
            continue
        require("libChrono_core" not in fields[5], "An external old Chrono backend was loaded")
        if "librobodyna_core.so" in fields[5]:
            mapped.add((fields[3], fields[4]))
    require(len(mapped) == 1, "Expected exactly one native core/factory owner")
    print(json.dumps({"schema": "robodyna.python_package_runtime.v1", "passed": True,
                      "package_manifest": str(manifest), "python": sys.version,
                      "native_core_instances": len(mapped), "checks": checks,
                      "physics_steps": 0, "gpu_calls": False}, indent=2))


if __name__ == "__main__":
    main()
