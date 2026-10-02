# Native retained DEM module

`//src/dem:dem` compiles the existing sphere/triangle DEM implementation: three
host C++ translation units and two CUDA translation units. `:visualization`
compiles its separate VSG adapter. The original files and notices remain under
`src/compatibility/chrono/src/chrono_dem`; none of their equations, precision,
stream operations, parameters or integration algorithms is changed by this batch.
This module is distinct from the optional TL-FEA DEM-Engine dependency.

`sources.bzl` is the explicit module inventory derived from the owning CMake
groups. Headers are exported separately; a broad source glob does not select
algorithms. Host C++ remains C++ compilation, and the two `.cu` files use NVCC
through the existing root CUDA toolchain. Architecture selection comes from the
root `--config=sm120` (or another explicitly qualified architecture) profile.

The retained CUDA backend definition is `CHRONO_USE_CUDA`. The unmodified
`ChConfigDem.h.in` becomes the module header without overriding the existing core
configuration. The backend links the selected `@rules_cuda//cuda:runtime` owner;
CUDA 13's nested CCCL headers come from the declared `@cuda_math//:cccl_headers`.
This does not change global `CHRONO_HAS_CUDA`, OpenMP or core numerical options.
CUDA compilation preserves the original compiler defaults for contraction,
division, denormals and streams; no fast-math or precision override is introduced.

Two link refinements are explicit. The root-selected runtime currently uses the
qualified shared cudart owner rather than introducing a separate static runtime.
The generic inherited CMake GPU helper also lists NVRTC, driver, cuBLAS and
cuSPARSE for every GPU module; retained DEM sources use only the CUDA runtime and
header-based CUB/Thrust APIs. Those unused math libraries are not new dependencies
of this native module. Host and kernel archives retain the complete module symbol
closure through `alwayslink`, avoiding a host/device static-archive back edge;
there is still exactly one owner for each original translation unit.

The source inventory test checks the CMake lists, all five original demo entry
points and the preintegration bytes recorded in `SourceBaseline.json`. That file
binds materialized source/data from the current qualified repository snapshot;
it does not replace the original import identities. The host contract test checks
public CUDA type selection and loads an original JSON configuration without
constructing a DEM system or calling CUDA.

Build and runtime qualification remain separate. Compilation/linking of
`//examples/dem:native_demos` must pass before these targets are recorded as built.
No GPU simulation has been run by the agent authoring this build batch. Actual
DEM runs need their own resource admission, working/data directory setup and
bounded duration; the inherited demonstration defaults are not smoke tests.
