"""Exercise actual optional Python modules without opening graphics or a GPU."""

import argparse
import gc
import importlib
import json
from pathlib import Path
import sys


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--package-manifest", type=Path, required=True)
    args = parser.parse_args()
    manifest = args.package_manifest.absolute()
    sys.path.insert(0, str(manifest.parent))
    public = importlib.import_module("robodyna")
    old = importlib.import_module("pychrono")
    modules = {}
    for name in ("core", "fea", "irrlicht", "vsg3d", "postprocess", "vehicle", "robot", "pardisomkl"):
        modules[name] = importlib.import_module("robodyna." + name)
        require(modules[name] is importlib.import_module("pychrono." + name), "Duplicated module identity: " + name)
    core, vehicle, robot, sparse = (modules[name] for name in ("core", "vehicle", "robot", "pardisomkl"))
    require(public.ChBody is old.ChBody is core.ChBody, "Public API does not reuse the actual core class")

    terrain = vehicle.FlatTerrain(1.25, 0.7)
    position = core.ChVector3d(0, 0, 0)
    require(terrain.GetHeight(position) == 1.25, "Original Vehicle terrain does not evaluate through the shared core types")
    require(abs(terrain.GetCoefficientFriction(position) - 0.7) < 1e-7, "Vehicle terrain friction differs")
    kinematics = robot.IndustrialKinematicsSCARA()
    require(kinematics.GetNumJoints() == 0, "Original robot kinematics default constructor differs")

    system = core.ChSystemSMC()
    system.SetNumThreads(1, 1, 1)
    solver = sparse.ChSolverPardisoMKL(1)
    system.SetSolver(solver)
    require(system.GetSolver().GetType() == core.ChSolver.Type_PARDISO_MKL,
            "Real Pardiso solver did not cross the shared System boundary")
    del solver
    gc.collect()
    require(system.GetSolver().GetType() == core.ChSolver.Type_PARDISO_MKL, "System lost its shared solver")
    require(system.GetChTime() == 0, "Import/constructor gate unexpectedly advanced mechanics")

    # Importing these generated native classes does not construct a visual
    # system, open a window, or initialize Vulkan/Irrlicht.
    for module, name in (("vsg3d", "ChVisualSystemVSG"), ("irrlicht", "ChVisualSystemIrrlicht"),
                         ("postprocess", "ChGnuPlot")):
        require(hasattr(modules[module], name), "Missing actual native exposure: " + module + "." + name)

    mapped = {}
    for line in Path("/proc/self/maps").read_text().splitlines():
        parts = line.split(maxsplit=5)
        if len(parts) != 6:
            continue
        require("libChrono_" not in parts[5], "An old external Chrono implementation was loaded")
        name = Path(parts[5]).name
        if name.startswith("librobodyna_"):
            mapped.setdefault(name, set()).add((parts[3], parts[4]))
    require(len(mapped.get("librobodyna_core.so", set())) == 1, "Expected one shared core/factory")
    require(len(mapped.get("librobodyna_native_images.so", set())) == 1, "Expected one shared STB owner")
    require(all(len(identities) == 1 for identities in mapped.values()), "A native module was loaded more than once")
    print(json.dumps({"schema": "robodyna.python_optional_runtime.v1", "passed": True,
                      "modules": sorted(modules), "native_owners": sorted(mapped), "numpy": False,
                      "physics_steps": 0, "graphics_initialized": False, "gpu_calls": False,
                      "checks": ["same public and compatibility module identities", "Vehicle/core value crossing",
                                 "actual Robot constructor", "Pardiso/System shared ownership", "one core and STB owner"]}, indent=2))


if __name__ == "__main__":
    main()
