"""Real core/FEA shared objects and coupled dynamics through one native backend."""

import argparse
import ctypes
import gc
import importlib
import importlib.machinery
import importlib.util
import json
import math
from pathlib import Path
import sys
import types


def near(require, actual, expected, tolerance=1e-12):
    require(math.isfinite(actual) and abs(actual - expected) < tolerance,
            f"Unexpected binding value {actual}; expected {expected} within {tolerance}")


def shared_mesh(core, fea, require):
    # The inherited core-only Mesh proxy has an unresolved node argument tag;
    # use the actual FEA interface, then cross the core System boundary. The
    # historical/current descriptor evidence is recorded in the workspace.
    mesh = fea.ChMesh()
    node = fea.ChNodeFEAxyz(core.ChVector3d(0, 0, 0))
    node.SetMass(2)
    mesh.AddNode(node)
    system = core.ChSystemSMC()
    system.SetNumThreads(1, 1, 1)
    system.AddMesh(mesh)
    alias = fea.CastToChMesh(system.GetMeshes()[0])
    require(alias.GetNumNodes() == 1, "Cross-module mesh alias lost its topology")
    node_alias = fea.CastToChNodeFEAxyz(fea.CastToChNodeFEAbase(alias.GetNode(0)))
    node_alias.SetMass(3)
    near(require, node.GetMass(), 3)
    del mesh, node
    gc.collect()
    require(alias.GetNumNodes() == 1, "System did not retain the shared mesh")
    system.RemoveMesh(alias)
    system.Clear()
    del alias, system
    gc.collect()
    near(require, node_alias.GetMass(), 3)


def coupled_spring(core, fea, require):
    system = core.ChSystemSMC()
    system.SetNumThreads(1, 1, 1)
    system.SetGravitationalAcceleration(core.ChVector3d(0, 0, 0))
    system.SetTimestepperType(core.ChTimestepper.Type_EULER_IMPLICIT_LINEARIZED)
    solver = core.ChSolverMINRES()
    solver.SetMaxIterations(100)
    solver.SetTolerance(1e-12)
    system.SetSolver(solver)
    mesh = fea.ChMesh()
    fixed = fea.ChNodeFEAxyz(core.ChVector3d(0, 0, 0))
    moving = fea.ChNodeFEAxyz(core.ChVector3d(1, 0, 0))
    fixed.SetFixed(True)
    moving.SetMass(1)
    spring = fea.ChElementSpring()
    spring.SetNodes(fixed, moving)
    spring.SetSpringCoefficient(12)
    spring.SetDampingCoefficient(0)
    mesh.AddNode(fixed)
    mesh.AddNode(moving)
    mesh.AddElement(spring)
    mesh.SetAutomaticGravity(False)
    system.AddMesh(mesh)
    body = core.ChBody()
    body.SetMass(2)
    body.SetInertiaXX(core.ChVector3d(1, 1, 1))
    body.SetSleepingAllowed(False)
    body.SetPos(core.ChVector3d(1.1, 0, 0))
    moving.SetPos(core.ChVector3d(1.1, 0, 0))
    system.AddBody(body)
    link = fea.ChLinkNodeFrame()
    require(bool(link.Initialize(moving, body)), "FE node/body constraint did not initialize")
    system.AddLink(link)
    dt, steps, peak_error = 1e-4, 1000, 0.0
    for _ in range(steps):
        require(bool(system.DoStepDynamics(dt, False)), "Coupled FE/body step failed")
        error = (body.GetPos() - moving.GetPos()).Length()
        peak_error = max(peak_error, error)
        require(math.isfinite(error) and error < 1e-9, "Coupled attachment drifted")
        violation = link.GetConstraintViolation()
        require(violation.Size() == 3 and all(abs(violation.GetItem(i)) < 1e-9 for i in range(3)),
                "Constraint residual changed")
    time = dt * steps
    expected = 1 + .1 * math.cos(2 * time)  # k=12 and combined moving mass=3.
    near(require, body.GetPos()[0], expected, 3e-6)
    near(require, moving.GetPos()[0], expected, 3e-6)
    near(require, system.GetChTime(), time, 1e-13)
    near(require, mesh.GetChTime(), time, 1e-13)
    near(require, body.GetChTime(), time, 1e-13)
    near(require, link.GetChTime(), time, 1e-13)
    reaction = link.GetReactionOnBody()
    node_reaction = link.GetReactionOnNode()
    require(reaction[0] < 0, "Body did not receive the spring's reaction")
    near(require, body.GetMass() * body.GetPosDt2()[0], reaction[0], 1e-8)
    require((reaction + node_reaction).Length() < 1e-12, "Link reactions are not equal and opposite")
    mesh_alias = fea.CastToChMesh(system.GetMeshes()[0])
    node_alias = fea.CastToChNodeFEAxyz(fea.CastToChNodeFEAbase(mesh_alias.GetNode(1)))
    body_alias = system.GetBodies()[0]
    del mesh, fixed, moving, spring, body
    gc.collect()
    require(mesh_alias.GetNumElements() == 1, "Coupled mesh lost its shared element")
    near(require, body_alias.GetMass(), 2)
    system.RemoveLink(link)
    system.RemoveMesh(mesh_alias)
    system.RemoveBody(body_alias)
    system.Clear()
    del link, mesh_alias, system
    gc.collect()
    near(require, node_alias.GetMass(), 1)
    near(require, body_alias.GetMass(), 2)
    return {"steps": steps, "time_s": time, "max_attachment_error": peak_error,
            "backend": "native_cpu", "cuda_dynamics": False}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("core-proxy", "core-extension", "fea-proxy", "fea-extension", "backend", "core-check"):
        parser.add_argument("--" + name, type=Path, required=True)
    args = parser.parse_args()
    paths = {name: path.resolve(strict=True) for name, path in vars(args).items()}
    spec = importlib.util.spec_from_file_location("_robodyna_core_checks", paths["core_check"])
    checks = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(checks)
    require = checks.require
    require(sys.implementation.name == "cpython" and sys.version_info[:2] == (3, 10), "Expected CPython 3.10 SDK")
    require("pychrono" not in sys.modules, "An unrelated binding package is already loaded")
    backend = ctypes.CDLL(str(paths["backend"]), mode=ctypes.RTLD_GLOBAL)
    # Package metadata only: the modules/classes below come from the declared
    # real generated proxies/extensions. No proxy or native type is synthesized.
    package = types.ModuleType("pychrono")
    package.__package__ = "pychrono"
    package.__spec__ = importlib.machinery.ModuleSpec("pychrono", None, is_package=True)
    package.__path__ = list(dict.fromkeys(str(paths[name].parent) for name in
                                        ("core_proxy", "core_extension", "fea_proxy", "fea_extension")))
    sys.modules["pychrono"] = package
    core = importlib.import_module("pychrono.core")
    fea = importlib.import_module("pychrono.fea")
    for module, name in ((core, "core_proxy"), (fea, "fea_proxy"),
                         (importlib.import_module("pychrono._core"), "core_extension"),
                         (importlib.import_module("pychrono._fea"), "fea_extension")):
        require(Path(module.__file__).resolve() == paths[name], "Imported an undeclared wrapper")
    checks.exercise(core)  # Existing body/inertia behavior must survive loading FEA.
    shared_mesh(core, fea, require)
    dynamics = coupled_spring(core, fea, require)
    gc.collect()
    mapped = set()
    for line in Path("/proc/self/maps").read_text().splitlines():
        fields = line.split(maxsplit=5)
        if len(fields) == 6:
            require("libChrono_core" not in fields[5], "Loaded an external second Chrono implementation")
            if "librobodyna_core.so" in fields[5]:
                mapped.add((fields[3], fields[4]))
    require(len(mapped) == 1 and backend._handle, "Expected one shared native backend")
    print(json.dumps({"schema": "robodyna.python_fea_runtime.v1", "passed": True,
                      "paths": {key: str(value) for key, value in paths.items()},
                      "native_backend_instances": len(mapped), "numpy": False, "dynamics": dynamics,
                      "checks": ["existing core behavior after FEA import", "cross-module Mesh/Node casts and ownership",
                                 "one-system FE/body constraint and reactions", "detached shared-object lifetime"]}, indent=2))


if __name__ == "__main__":
    main()
