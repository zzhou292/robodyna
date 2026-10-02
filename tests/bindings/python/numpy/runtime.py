"""Exercise the retained real NumPy C-API bridge and its copy/owner semantics."""

import argparse
import gc
import importlib
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).parent.parent))
from origin import require_declared_origin


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--package-manifest", type=Path, required=True)
    args = parser.parse_args()
    manifest = args.package_manifest.absolute()
    document = json.loads(manifest.read_text())
    require(document["numpy"] is True, "Expected the separate NumPy-enabled wrapper profile")
    sys.path.insert(0, str(manifest.parent))
    product = importlib.import_module("robodyna")
    core = importlib.import_module("robodyna.core")
    require(core is importlib.import_module("pychrono.core"), "Public and compatibility core differ")
    np = importlib.import_module("numpy")
    require(np.__version__ == "1.26.4", "Imported an unqualified NumPy version")
    require_declared_origin(np, manifest, document, "numpy/__init__.py")

    vector = product.ChVector3d(1, 2, 3)
    values = vector.to_numpy()
    require(values.dtype == np.float64 and values.shape == (3,), "Vector array dtype/shape changed")
    require(np.array_equal(values, [1, 2, 3]), "Vector array values changed")
    values[0] = 99
    require(vector.x == 1, "to_numpy unexpectedly borrowed mutable native vector storage")
    q = product.ChQuaterniond(1, 2, 3, 4).to_numpy()
    require(q.dtype == np.float64 and q.shape == (4,) and np.array_equal(q, [1, 2, 3, 4]), "Quaternion bridge differs")

    for shape, constructor in [((3, 3), product.ChMatrix33d), ((4, 5), product.ChMatrixDynamicd)]:
        original = np.arange(shape[0] * shape[1], dtype=np.float64).reshape(shape) / 4
        native = constructor(original)
        expected = original.copy()
        original[:] = -9
        exported = native.to_numpy()
        require(exported.dtype == np.float64 and exported.shape == shape, "Matrix array dtype/shape changed")
        require(np.array_equal(exported, expected), "Native matrix did not own its imported array values")
        exported[:] = 17
        del original, exported
        gc.collect()
        require(np.array_equal(native.to_numpy(), expected), "Exported-array lifetime changed native matrix storage")

    backends = set()
    for line in Path("/proc/self/maps").read_text().splitlines():
        fields = line.split(maxsplit=5)
        if len(fields) == 6 and "librobodyna_core.so" in fields[5]:
            backends.add((fields[3], fields[4]))
    require(len(backends) == 1, "Expected one shared native core after importing NumPy")
    print(json.dumps({"schema": "robodyna.python_numpy_runtime.v1", "passed": True,
                      "numpy": np.__version__, "numpy_origin": np.__file__, "native_core_instances": len(backends),
                      "checks": ["actual C-API vector/quaternion/matrix conversions", "input and output copy ownership", "declared runtime origin"],
                      "physics_steps": 0, "gpu_calls": False}, indent=2))


if __name__ == "__main__":
    main()
