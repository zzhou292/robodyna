"""Exercise the real CPython core wrapper and its single native backend."""

import argparse
import ctypes
import gc
import importlib
import json
import math
from pathlib import Path
import sys


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def near(actual, expected):
    require(math.isfinite(actual) and abs(actual - expected) < 1e-12,
            f"Unexpected numerical binding result: {actual} versus {expected}")


def exercise(core):
    body = core.ChBody()
    body.SetMass(7.25)
    body.SetTag(173)
    copy = core.ChBody(body)
    copy.SetMass(3.5)
    near(body.GetMass(), 7.25)
    near(copy.GetMass(), 3.5)

    aux = core.ChBodyAuxRef()
    aux.SetFrameCOMToRef(core.ChFramed(core.ChVector3d(.25, -.5, .75)))
    # GetFrameCOMToRef returns an owned frame value; GetPos exposes a borrowed
    # vector. Retain the frame while reading that vector through the old bindings.
    aux_frame = aux.GetFrameCOMToRef()
    near(aux_frame.GetPos()[0], .25)
    box = core.ChBodyEasyBox(2, 4, 6, 3, False, False)
    near(box.GetMass(), 144)
    for index, expected in enumerate((624, 480, 240)):
        near(box.GetInertiaXX()[index], expected)

    system = core.ChSystemNSC()
    system.SetNumThreads(1, 1, 1)
    system.AddBody(aux)
    system.AddBody(box)
    require(len(system.GetBodies()) == 2, "Body container did not retain its elements")
    alias = system.GetBodies()[0]
    alias.SetMass(4.5)
    near(aux.GetMass(), 4.5)
    derived = core.CastToChBodyAuxRef(alias)
    derived_frame = derived.GetFrameCOMToRef()
    near(derived_frame.GetPos()[2], .75)
    del aux, derived
    gc.collect()
    near(alias.GetMass(), 4.5)
    system.RemoveBody(alias)
    near(alias.GetMass(), 4.5)
    system.Clear()

    marker = core.ChMarker()
    marker.SetName("binding marker")
    force = core.ChForce()
    force.SetName("binding force")
    body.AddMarker(marker)
    body.AddForce(force)
    # SetMforce updates body-relative force state and requires an attached body.
    force.SetMforce(12.5)
    del marker, force
    gc.collect()
    require(len(body.GetMarkers()) == 1 and len(body.GetForces()) == 1,
            "Body did not retain shared child objects")
    marker = body.GetMarkers()[0]
    force = body.GetForces()[0]
    require(marker.GetName() == "binding marker" and force.GetName() == "binding force",
            "Child aliases changed their objects")
    near(force.GetMforce(), 12.5)
    body.RemoveAllMarkers()
    body.RemoveAllForces()
    require(marker.GetName() == "binding marker" and force.GetName() == "binding force",
            "Detached child proxy lost its shared ownership")
    # Parent GetBody wrappers are deliberately not used here. The inherited
    # ChMarker getBodySP creates a new control block from a raw parent pointer;
    # that pre-existing ownership behavior requires an isolated investigation.

    tensor = core.ChMatrix33d(core.ChVector3d(2, 3, 4))
    shifted = core.ChInertiaUtils.TranslateInertia(tensor, core.ChVector3d(2, 0, 0), 3)
    for index, expected in enumerate((2, 15, 16)):
        near(shifted.getitem(index, index), expected)
    composite = core.CompositeInertia()
    composite.AddComponent(core.ChFramed(), 2, tensor)
    near(composite.GetMass(), 2)
    near(composite.GetCOM().Length(), 0)
    properties = core.ChMassProperties()
    properties.mass = 2
    properties.com = core.ChVector3d(1, 2, 3)
    properties.inertia = tensor
    near(properties.mass, 2)
    near(properties.com[1], 2)
    near(properties.inertia.getitem(2, 2), 4)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--proxy", type=Path, required=True)
    parser.add_argument("--extension", type=Path, required=True)
    parser.add_argument("--backend", type=Path, required=True)
    args = parser.parse_args()
    require(sys.implementation.name == "cpython" and sys.version_info[:2] == (3, 10),
            "The qualified extension profile requires CPython 3.10")
    proxy, extension, backend = (path.resolve(strict=True) for path in (args.proxy, args.extension, args.backend))
    # Load the explicitly supplied DSO once. Extension DT_NEEDED resolves this
    # same SONAME; no old Chrono installation or second static core is loaded.
    native_backend = ctypes.CDLL(str(backend), mode=ctypes.RTLD_GLOBAL)
    sys.path[:0] = [str(proxy.parent), str(extension.parent)]
    core = importlib.import_module("core")
    native = importlib.import_module("_core")
    require(Path(core.__file__).resolve() == proxy and Path(native.__file__).resolve() == extension,
            "Imported a different installed core wrapper")
    exercise(core)
    gc.collect()
    mapped = set()
    for line in Path("/proc/self/maps").read_text().splitlines():
        fields = line.split(maxsplit=5)
        if len(fields) == 6 and "librobodyna_core.so" in fields[5]:
            mapped.add((fields[3], fields[4]))  # Device and inode identify one DSO across mappings.
    require(len(mapped) == 1 and native_backend._handle, "Expected one loaded native core implementation")
    print(json.dumps({"schema": "robodyna.python_core_runtime.v1", "passed": True,
                      "python": sys.version, "proxy": str(proxy), "extension": str(extension),
                      "backend": str(backend), "native_backend_instances": len(mapped),
                      "checks": ["body construction/copy", "derived constructors/casts", "shared body containers",
                                 "marker/force attachment and child lifetime", "canonical inertia operations"],
                      "numpy": False, "physics_steps": 0,
                      "excluded": ["unsafe inherited parent GetBody ownership helpers", "managed C# runtime"]}, indent=2))


if __name__ == "__main__":
    main()
