# Conventional shell qualification sources

This opt-in TL-FEA test project keeps the maintained conventional Q4 CUDA
fixtures, independent native oracles, exact donor sources, and source provenance
inside the repository. It is a numerical qualification boundary. It does not
provide a production element, crash solver, or substitute simulation executable.
OpenRadioss supplies selected source routines; its full driver is not built.

| Directory | Maintained boundary | Captured pre-migration gate |
| --- | --- | --- |
| `operators/` | Bounded K1 geometry/K3 prescribed-resultant assembly; one fresh elastic K2 increment | 16 S1 and 13 S2a GPU tests |
| `hourglass/` | Fixed-shape planar stabilization with owned accepted/trial history | 17 S2b GPU tests |
| `native/chvis3/` | Exact native CHVIS3 plus explicit C ABI/context adapters | Six native tests |
| `reference/cuda/` | Eight immutable selected source/header/license files and manifest | Pinned hashes and exact original bytes |
| `operators/generated/` | Precisely generated NPT3 Gauss integration adapter | Eight existing generator checks |

Migration adds two source-integrity regression checks, bringing the generator
suite to ten. The numerical test sources and fixture arithmetic are copied
byte-for-byte. Passing results above belong to the frozen workspace checkpoints;
rebuilding this relocated project must reproduce those gates before migration
is accepted. Root records new build/test reports outside the source tree.
`persistent/` and `native/phase/` are separately owned additions; their existence
or compilation alone is not a passing persistent-shell gate.

The modular boundaries remain explicit: immutable configuration, borrowed
kinematics, owned accepted/trial state, element results, and transactional
publication. C++ interfaces expose fixture value types, not donor structs.
Donor kernels stay private to CUDA translation units. Native COMMON context is
confined to the optional numerical oracle; it is not a production dependency.
The current fixtures allocate/read back tiny batches to inspect physical
invariants, so they establish no throughput or production device-residency claim.

## Integrity and migration provenance

`migration-manifest.json` records each copied source path, frozen workspace
origin, original SHA256, local SHA256, and whether changes are limited to build,
path, documentation, or provenance plumbing. Exact numerical copies must retain
the original hash. Its inventory covers this migration, not future separately
owned phase/persistent files. The top-level CMake/README are new integration
files. Generated manifest/checksum metadata are regenerated from the relocated
script and are checked by that generator rather than circularly fingerprinted.

`operators/generate_gauss3.py --verify` checks the migration inventory, all
original CUDA donor hashes, the existing TL quadrature table hash, the exact
three source edits, the generated source/header/patch/license, and generated
metadata. Configuration and builds require that read-only check. The native
oracle additionally verifies its own pinned 21-file closure and git blob IDs.
Builds do not need the old `crash-work` source copies or a reference checkout.
The frozen workspace files remain unchanged for historical comparison.

The original AGPL-3.0-or-later notices and complete license are preserved with
each source closure. The OpenRadioss revision is
`a62b27e6baa555d222a580d6218867d0be4d70b5`. The Gauss change uses TL's existing
`lib_utils/quadrature_utils.h`; the original midpoint NPT3 bending defect stays
an exact 8/9 diagnostic regression. Existing midpoint history is never relabelled
as Gauss history. The relocated generator changes paths and integrity plumbing;
it does not change the numerical transformation or generated CUDA/header bytes.
The historical network staging utility is omitted: all required originals are
already present, and no build performs downloads or installs a compiler.

## Focused build and verification

Run from the TL-FEA repository root. Use the existing workstation guard and a
separate build/report location. Root coordinates all commands in this session;
these recipes do not grant a concurrent build slot. Supply the actual compatible
CUDA architecture/compiler rather than assuming the example hardware elsewhere.

```sh
python3 tools/run_bounded.py --report ../crash-work/reports/tl-shell-configure.json --cpus 2 --max-rss-gib 2 --timeout 120 -- cmake -S lib_utest/qualification/shell -B ../crash-work/build/tl-shell-qualification -DCMAKE_CUDA_COMPILER=/usr/local/cuda/bin/nvcc -DCMAKE_CUDA_ARCHITECTURES=120 -DCMAKE_BUILD_TYPE=RelWithDebInfo
python3 tools/run_bounded.py --report ../crash-work/reports/tl-shell-build.json --cpus 2 --max-rss-gib 2 --timeout 180 -- cmake --build ../crash-work/build/tl-shell-qualification --parallel 1
```

Default configuration uses C++17 and CUDA17, system CUDAToolkit, GTest and Python.
It has no Fortran requirement. For the complete captured 46-GPU-test/six-native-
test gate, configure with these additional explicit options before building:

```sh
-DTL_SHELL_ENABLE_NATIVE_ORACLES=ON -DCMAKE_Fortran_COMPILER="$PWD/../crash-work/tools/gfortran-11.4.0/gfortran-local"
```

This option builds the private `chvis3_native` oracle and enables hourglass
CPU/GPU comparisons. `TL_SHELL_ENABLE_PHASE_ORACLE` is separately opt-in and
requires the separately staged phase source; it also enables the native context.
When CHVIS3 and phase libraries coexist they share native COMMON symbols, so
serial test coordination is required across both wrappers.

The tracked `native/phase` source has now passed 11 CPU tests. Enable
`TL_SHELL_ENABLE_PERSISTENT=ON` for the preallocated evolving-geometry/history
fixture. With both phase and persistent options enabled, its 12 GPU tests
include two direct native phase comparisons. Details, ownership and remaining
physical gates are in [persistent/README.md](persistent/README.md).

The persistent operator tests pass, including changing geometry, repeated
stress/thickness history and rejection after each stage. Rigid secant-path
diagnostics report nonzero stress/force/work with refinement; XML explicitly
sets `production_objectivity_qualified=false`. These passing characterization
tests do not qualify the element for production finite-rotation dynamics.

```sh
python3 tools/run_bounded.py --report ../crash-work/reports/tl-shell-integrity.json --cpus 1 --max-rss-gib 0.25 --timeout 30 -- python3 lib_utest/qualification/shell/operators/generate_gauss3.py --self-test --verify
python3 tools/run_bounded.py --report ../crash-work/reports/tl-shell-tests.json --cpus 2 --max-rss-gib 1 --timeout 120 --gpu 0 --max-gpu-growth-gib 1 -- ctest --test-dir ../crash-work/build/tl-shell-qualification --output-on-failure -j 1
```

CTest tests are serialized and bounded. Explicit fixture allocations peak at
19,584 device bytes; CUDA context/runtime allocation is additional. Generic
`CudaError` is an operation failure, not a production fatal-device/context-
recovery contract. Finite-input overflow, invalid input, rejected trials,
stale ownership and successful retry are separately tested.

Preserve report names for captured runs. If deliberately relocating or updating
the generator, regenerate its metadata with `operators/generate_gauss3.py`
without `--verify`, then review source hashes and run verification; normal
configuration/builds only verify and never rewrite source files.
