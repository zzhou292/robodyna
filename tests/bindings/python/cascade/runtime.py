"""Qualify real bidirectional CAD shape exchange with one native OCCT owner."""

import argparse
import gc
import importlib
import json
from pathlib import Path
import sys


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def near(actual, expected, tolerance, message):
    require(abs(actual - expected) <= tolerance, message)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--package-manifest", type=Path, required=True)
    args = parser.parse_args()
    manifest = args.package_manifest.absolute()
    document = json.loads(manifest.read_text())
    sys.path.insert(0, str(manifest.parent))
    product = importlib.import_module("robodyna")
    core, cascade = product.core, product.cascade
    from OCC.Core import BRepPrimAPI

    # Match the already-qualified native CAD coupon's independently known box.
    maker = BRepPrimAPI.BRepPrimAPI_MakeBox(1.0, 2.0, 3.0)
    shape = maker.Shape()
    require(not shape.IsNull(), "PythonOCC did not produce a real box")
    first = cascade.ChBodyEasyCascade(shape, 7.0, False, False)
    near(first.GetMass(), 42.0, 1e-10, "PythonOCC-to-CAD native mass differs")
    inertia = first.GetInertiaXX()
    for value, expected in [(inertia.x, 45.5), (inertia.y, 35.0), (inertia.z, 17.5)]:
        near(value, expected, 1e-9, "PythonOCC-to-CAD native inertia differs")

    # The field view borrows first's storage, so keep first alive while using it.
    # The second native body then takes its own OCCT shared-shape handle.
    returned = first.topods_shape
    require(not returned.IsNull(), "Native-returned shape is not a real PythonOCC object")
    require(returned.IsSame(shape), "Bidirectional shape conversion changed its CAD identity")
    second = cascade.ChBodyEasyCascade(returned, 7.0, False, False)
    del returned, shape, maker, first
    gc.collect()
    near(second.GetMass(), 42.0, 1e-10, "Native body lost the copied CAD shape lifetime")
    require(not second.topods_shape.IsNull(), "Retained CAD shape became null")

    system = core.ChSystemNSC()
    system.SetNumThreads(1, 1, 1)
    system.SetGravitationalAcceleration(core.ChVector3d(0, 0, 0))
    system.AddBody(second)
    before = second.GetPos().x
    second.SetPosDt(core.ChVector3d(1, 0, 0))
    require(system.DoStepDynamics(0.001), "CAD body failed its one real native System step")
    near(second.GetPos().x, before + 0.001, 1e-12, "CAD/core physical body state is not shared")
    near(system.GetChTime(), 0.001, 1e-15, "CAD/core clock differs")

    anchors = [manifest.parent / relative / ".robodyna-runtime.json"
               for relative in document["runtime_python_roots"]]
    metadata = next(json.loads(path.read_text()) for path in anchors
                    if path.is_file() and json.loads(path.read_text()).get("kind") == "pythonocc")
    expected_root = Path(metadata["occt_root"]).resolve()
    owners = {}
    core_owners = set()
    for line in Path("/proc/self/maps").read_text().splitlines():
        fields = line.split(maxsplit=5)
        if len(fields) != 6:
            continue
        filename = Path(fields[5])
        require("libChrono_" not in filename.name, "Old external Chrono backend was loaded")
        if filename.name == "librobodyna_core.so":
            core_owners.add((fields[3], fields[4]))
        if filename.name.startswith("libTK"):
            require(filename.resolve().is_relative_to(expected_root), "An independent OCCT installation was loaded")
            owners.setdefault(filename.name, set()).add((fields[3], fields[4]))
    require(len(core_owners) == 1, "Expected one native core/factory owner")
    require(owners and all(len(value) == 1 for value in owners.values()), "Duplicate CAD native owner")
    print(json.dumps({"passed": True, "physics_steps": 1, "occt_libraries": sorted(owners),
                      "checks": ["real bidirectional TopoDS shape exchange", "known box mass/inertia",
                                 "copied CAD shape lifetime", "single core step", "same OCCT SDK mapped once"],
                      "scope": "Actual CAD/native ownership and physical body gate; no GUI or full original demo claim"}, indent=2))


if __name__ == "__main__":
    main()
