# LAW90 preparation qualification

Five host functions cover original pre/post moduli, reader defaults, distinct
directional knot slopes and signed/zero/extrapolated ordinates, complete late
curve rejection/cursor rollback/retry, and rejection of other native branches.
The original28-point fixture retains raw MPa and exactly once-converted SI
binary64 values, original card records and source/canonical receipt hashes.

Four native functions are authored for root execution: all33 preparation fields
and reader defaults; separate original t/mm/s versus SI comparison; every knot
from every valid cursor plus both endpoint extrapolations; and recurrent
reversal with signed zero/negative slopes. Same-domain comparisons are bitwise.
The separate working-unit comparison uses2e-14 relative/absolute scale and
explicit wrong-E/contact-coefficient rejection controls. That comparison is
not used for same-domain preparation or interpolation.

One CUDA function prepares the actual material and carries its own curve cursor
through58 queries, then injects a late bad point and retries. Its preparation
fields and trajectory are compared with the independent native packet. No
global CUDA stack reservation or large per-thread scratch is introduced.

The native harness authenticates the complete enclosing HM_READ_MAT90 source,
including its actual unmodified flag lines109–111, cutoff defaults137–139 and
the complete scale/default/parameter region150–213. The wrapper supplies
post-HM_GET values and one real table; it does not port the reader equations.
Full FUNC_SLOPE, LAW90_UPD and VINTER/VINTER2/VINTER2DP/FINTER2 bodies compile
unchanged except private namespaces. LAW90_UPD's warning uses an I/O-only
module stub; unexpected errors stop. Original includes/constants/precision
modules are retained by shared immutable source-manifest paths. None of these
native routines calls the C++ implementation.

The original default export/read into HM_GET remains separate. This harness
does not establish source KCON semantics, point stress/history recurrence,
ET/SSP force-caller behavior, geometry, mass or app admission.

Owning CMake:

```
cmake -S lib_utest/qualification/law90_preparation -B <build> \
  -DLAW90_NATIVE=ON -DLAW90_CUDA=ON -DCMAKE_CUDA_ARCHITECTURES=120
cmake --build <build> -j1
ctest --test-dir <build> --output-on-failure
```

CTest: `law90_preparation_host`, `law90_preparation_identity`,
`law90_preparation_native`, `law90_native_identity`,
`law90_preparation_cuda`. Bazel:
`//lib_src/materials/law90:preparation` and
`//lib_utest/qualification/law90_preparation:host_check`.
No existing numerical material target changes; the only shared production
delta is the additive VINTER2 helper and its own Bazel target.

Author runs only bounded host compilation/testing and C++/source syntax checks.
Root owns actual Fortran/native/CUDA execution. The first author host report
preserves an incorrect exact-knot-equals-stored-ordinate expectation; the fixed
test compares unscaled lookup across different preparation scales, leaving
all production interpolation expressions unchanged.

Author evidence: `crash-work/reports/law90-preparation-author-tests-2` records
all5 host functions passing; `...-identity-1`, `...-syntax-1`, and
`...-configure-1` record source authentication, native/CUDA-shaped C++ syntax,
and host-only CMake configuration. No Fortran compiler or device test was run
by the author.
