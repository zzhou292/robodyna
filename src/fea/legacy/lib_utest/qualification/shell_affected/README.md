# Combined affected-shell qualification

This CMake entrypoint composes the existing resident failure, mapped Q/T,
parent activity, and mapped QEPH gather projects in one build graph. Production
and native libraries are compiled once per shared target. Every test executable,
source verifier, native preparation dependency, numerical flag, and test command
remains owned by its original project. No previously built archive is imported.
Build-time improvement has not been measured.

The only changes to existing files are target-existence guards in
`shell_parent_activity/CMakeLists.txt` and `qeph_mapped_gather/CMakeLists.txt`.
Their standalone behavior and options remain unchanged. These two files are not
records in the existing qualification source manifests; no donor or mechanics
receipt is refreshed for this change. The integration baseline is `dfa0a93`.

## Coverage and target ownership

`ExpectedTests.txt` is the distinct union of the existing Release/CUDA-120 caches
`resident-shell-failure-root-1` (19), `qt-mapped-root-1` (25),
`shell-parent-activity-root-1` (1), and `qeph-mapped-gather-root-1` (3).
Their 48 registrations contain 43 distinct tests. The five duplicated native
tests are `qeph_force_check`, `qeph_kinematics_check`, `t3_force_check`,
`t3_kinematics_check`, and `t3_startup_check`.

Configure requires every member of this union and rejects duplicate names. It
does not remove or filter newly added tests. The combined entrypoint enables the
complete native/CUDA child options, including native test registration, even if
its cache previously disabled one. The original leaf defaults are unchanged.

Resident failure must come first: its T3 project creates complete native R1/R2/R3
and QEPH owners. Mapped Q/T then reuses their existing Fortran module directories,
prepared sources, compiler options, and symbol definitions. Native preparation
still verifies its pinned sources at configure/build time. The final two children
reuse the already-owned nodal and batch targets. An existing startup-only T3
target is insufficient; the native layered caller deliberately rejects it.

Keep the executables separate. The activity and failure tests have different
`cudaMemcpyAsync` fault wrappers, and native COMMON state is process-local.
Keep mapped Q/T's direct host compilation of `Startup.cpp`; this small independent
host qualification boundary is intentional.

T3 activity readback is excluded from this first composition. Its CUDA child
adds `physical_publication`, whose unconditional owner/batch creation would need
another target-reuse change. Run that existing qualifier separately.

## Root qualification

Run under the existing workstation/heavy-job guard. Configuration itself enables
CUDA and Fortran and therefore belongs to the root hardware/toolchain lane.
Use a fresh cache and the same toolchain as the four original gates:

```sh
TL_CHECKOUT=/home/jsonzhou/Desktop/chrono-work/Total-Lagrangian-FEA
SHELL_AFFECTED_BUILD=/home/jsonzhou/Desktop/chrono-work/crash-work/build/shell-affected-root-1
cmake -S "$TL_CHECKOUT/lib_utest/qualification/shell_affected" \
  -B "$SHELL_AFFECTED_BUILD" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_CXX_COMPILER=/usr/bin/c++ \
  -DCMAKE_CUDA_COMPILER=/usr/local/cuda/bin/nvcc \
  -DCMAKE_CUDA_ARCHITECTURES=120 \
  -DCMAKE_Fortran_COMPILER=/home/jsonzhou/Desktop/chrono-work/crash-work/tools/gfortran-11.4.0/gfortran-local
cmake --build "$SHELL_AFFECTED_BUILD" --parallel 1
ctest --test-dir "$SHELL_AFFECTED_BUILD" --show-only=json-v1
ctest --test-dir "$SHELL_AFFECTED_BUILD" --output-on-failure -j1
```

Build `all`, as above, to retain inherited native and regression executables.
Expected initial inventory: 43 tests, all present exactly once. The existing
mapped Q/T and gather source-identity tests remain in that inventory; native and
T3 verification dependencies remain attached to their original targets. A full
configure/build/test pass is still required before claiming combined runtime
qualification. Compare later header-change rebuilds with the four-cache workflow
under the same resource limits before reporting a speedup.
