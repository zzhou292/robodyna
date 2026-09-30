# Native symmetric spectrum gate

This isolated gate qualifies the native-ordered spectral prerequisite for
LAW90 history. It does not implement the material point or admit the radiator.
Author scope is host and syntax under1CPU/512MiB. Root runs native/CUDA.

The complete enclosing SIGEPS33 is reused from the existing LAW42 native
fixture, SHA256
`f898d60a29ab38d0c4fc2773d83a68620f1accba15838e3a9a5da13c2a9c6dbf`.
The manifest also binds complete `precision.c`, constant/precision modules
and original selected binary64 includes. `prepare_sources.py` extracts the
complete `VALPVEC_V` body, preserving all native vector queue loops. Only
private symbol names change. The wrapper tests both mixed batches and scalar
calls, so the production scalar decomposition is not its oracle.

Complete engine SIGEPS90 is retained and authenticated separately, SHA256
`e1bd7623d72d25d952b59f733298ca235b46c8b1b92e40220da193d4f812d0a5`.
Its exact294–318 projection region receives the native spectrum and supplied
engineering rate. This observation is independent of the C++ test projection;
it does not execute or claim a full material recurrence. The C ABI returns15
fields: values3, row-major direction matrix9, projected rates3. The native
constant ABI independently exposes FLMIN, FLM, EM10 and EM20.

The196 packets include zero/hydrostatic, rotations, compression exchange,
repeated and nearly repeated roots, both sides of the near-triple threshold,
small/large scales and a129-packet varying path. Native comparisons preserve
vector order and signs; there is no absolute-value or sorted-eigenpair escape.
Comparison factor2e-12 uses the tensor norm for eigenvalues,1 for unit-vector
components and10 for this prescribed rate tensor. Wrong basis and rate
perturbations are explicit negative controls. The well-conditioned analytic
host reconstruction check is separate; native fallback is not falsely judged
as exact reconstruction. CUDA uses the same independent native packets and
checks device-owned late invalid output preservation/retry bitwise.

Author results:4 host functions and complete identities pass through owning
host CMake (`native-spectrum-author-cmake-tests-1`). Native C++ and CUDA-shaped
host syntax also pass; the latter does not replace NVCC/device execution.
The first host compile failure was a missing local
HD macro after the existing quaternion header undefines its private macro;
the failed report is retained, and the adapter now owns a uniquely named
macro. No native/GPU execution is claimed at author freeze.

Root qualification:

```sh
cmake -S lib_utest/qualification/native_symmetric_eigen3 -B BUILD \
  -DSPECTRUM_NATIVE=ON -DSPECTRUM_CUDA=ON -DCMAKE_CUDA_ARCHITECTURES=120
cmake --build BUILD -j1
ctest --test-dir BUILD --output-on-failure
```

CTest groups: `native_spectrum_host` (4 functions), `native_spectrum_reference`
(3), `native_spectrum_cuda` (2), `native_spectrum_identity`, and
`symmetric_spectrum_native_identity`. Bazel targets:
`//lib_src/math:native_symmetric_eigen3` and
`//lib_utest/qualification/native_symmetric_eigen3:host_check`.
The existing generic spectrum and LAW42 production are unchanged; there is no
affected material target from this additive adapter. Preparation dependency
3405b20 is included unchanged in the worktree.
