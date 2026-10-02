# Native retained SPH solver

The public native owner is `//src/sph:sph`; its VSG adapter is
`//src/sph/visualization:vsg`. The generic coupling owner remains separate under
`//src/coupling/fsi:fsi`. All consume the existing core implementation rather than
linking a second installed Chrono library.

The source partition follows the actual local CMake recipe:

- Three generic FSI files compile as host C++ in the coupling owner.
- **Five `.cpp` files and twelve `.cu` files compile as CUDA**, exactly matching
  `FSISPH_GPU_SOURCE_FILES`. The rules_cuda NVCC action uses `-x cu`; nominal file
  extensions do not accidentally demote those five files to ordinary C++.
- The CUDA profile compiles the one VSG adapter as host C++, matching CMake. The
  alternative HIP treatment is not selected here.
- The two STB sources retain their existing `native_stb` owner.

The initial profile preserves the CMake default `CH_USE_SPH_DOUBLE=OFF`: SPH
`Real` is float, while generic mechanical body states remain double precision.
`ChFsiConfigSPH.h` is generated from the retained template. No Splashsurf tool has
been admitted, so reconstruction support remains disabled as in that CMake
configuration; the original optional implementation is retained unchanged.

`CHRONO_USE_CUDA` and the selected runtime/header interface are public because
ordinary consumers need the CUDA types in the public API. CUDA-device and
CPP-host Thrust definitions remain **private to SPH implementation targets**, as
does `CH_API_COMPILE_FSI`. This avoids imposing a device policy on other modules,
including Multicore's own Thrust profile. The CUDA relaxed-constexpr option is
retained; no fast-math, contraction, denormal, time-step or stream-mode override
is introduced. Device symbols remain in their original translation units.

The native build uses the root-selected `@rules_cuda//cuda:runtime` and declared
CUDA 13 CCCL headers. It does not add another cudart implementation. The generic
inherited CMake GPU helper lists NVRTC/driver/cuBLAS/cuSPARSE for all GPU modules;
the admitted SPH files do not call those APIs. The old BiCGStab/GMRES files that
would use vendor sparse/BLAS APIs are commented out in the owning source list and
are intentionally not activated by a compile glob. This link-closure refinement
does not rewrite the active algorithms.

`SourceBaseline.json` authenticates 86 existing source/header/demo inputs before
this build integration. The source tests verify the exact CUDA language partition
and complete 24-case FSI roster. A separate Bazel analysis test inspects actual
compile owners and exported definitions, so intended inventories cannot hide a
duplicate implementation or leaked private Thrust policy. The host-profile test
only inspects public types and values; it constructs no fluid system, calls no
CUDA function and advances no clock.

Compilation and runtime qualification remain separate. Root qualification must
compile `//examples/sph:native_demos` and run the source/ownership/header gates.
Actual bounded GPU coupons and case-specific conservation/convergence tests are
later runtime gates; this declaration batch is not numerical validation or a
performance comparison. The authors of this batch launched no heavy build or GPU
simulation and changed no original algorithm or coupling-clock source.
