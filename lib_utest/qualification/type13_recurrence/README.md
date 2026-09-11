# TYPE13 native H1 recurrence qualification

Scope and dimensional/phase contract: `lib_src/elements/type13/RECURRENCE.md`.
The property uses the already qualified literal original MAT100 converter values;
this target does not parse a source keyword or run an attached vehicle.

Author checks: six host functions PASS, native oracle/test and affected TYPE25
host sources pass C++ syntax, and all donor/extract hashes pass. Native/Fortran
and CUDA execution remain root-scheduled qualification; they were not run by the
author. The independent native tests carry their own complete channel histories.
They exercise:

- All six channels through load/hold/unload/reversal/reload on a dense frame.
- Original first, last and closest-N3-alignment beam references.
- TT=0 versus TT>0 reference initialization, exact failure threshold, retained
  current force and the next zero-force/previous-force work contribution.
- Curve knots in both search directions and extrapolation.
- A prescribed 96-packet combined motion with nonzero mean spin, all six yielded
  histories and a wrong-midpoint-velocity negative control.
- Rejected late history followed by retry along the independent native history.

Six host functions cover staged startup, actual yielding in every mode, wrench
balance, late sixth-channel arithmetic rejection, alias retry, rate overflow and
collapsed geometry. Two optional CUDA functions retain their own device history
and prove a rejected packet preserves output then exactly retries its clean
device result. Failure-limit fixtures explicitly use smaller resolved limits to
reach the same native branch; the original declaration remains unchanged.

Root owning gate:

```sh
cmake -S TL_SOURCE/lib_utest/qualification/type13_recurrence -B BUILD \
  -DTYPE13_H1_NATIVE_CHECKS=ON -DTYPE13_H1_CUDA_CHECKS=ON \
  -DCMAKE_Fortran_COMPILER=EXISTING_GFORTRAN -DCMAKE_CUDA_ARCHITECTURES=120
cmake --build BUILD --parallel 1
ctest --test-dir BUILD --output-on-failure
```

Targets: `type13_h1_host_test`, `type13_h1_native_test`, `type13_h1_cuda_test`;
six host + six native + two CUDA functions and two independent identity checks.
Also run the owning `qualification/type25` host/native/CUDA gate because the
shared frame/projection moved into small reusable headers. Its native wrappers,
TYPE25 response/deformation/stability equations and admission are unchanged.
Existing `qualification/type13` startup/native/CUDA remains a separate gate.
Bazel owns `//lib_utest/qualification/type13_recurrence:type13_h1_check` and
`:type13_h1_cuda_check`; Fortran remains in its existing CMake qualification path.
