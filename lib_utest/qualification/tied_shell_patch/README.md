# Tied shell patch qualification

The native oracle computes its own DPARA directly from geometry and calls the
complete selected I2VIROT3 motion routine. Complete pinned source files, exact
force/geometry fragments and their byte identities are retained under `native`.
Test packet code supplies explicit IRODDL1/WEIGHT1 context; it does not seed
native expected values with production cofactors or recovered motion.

Nine functions pass in the owning root build `tied-patch-root-1`:

- Four host functions check force and moment balance, virtual power, rigid
  motion, true repeated-node triangle weighting, invalid input and rollback.
- Three native functions cover four shapes and sixteen load/motion packets,
  off-surface pure couples, and the repeated-node motion indexing control.
- Two actual CUDA functions compare native results and check late nonfinite
  rejection followed by exact retry. Successful comparisons use named fields;
  raw object bytes are used only to prove rejected calls did not write output.

Reports are `crash-work/reports/tied-patch-root-{configure,build,tests}-*.json`;
XML is in `tied-patch-root-functions-1`. All nine functions have zero failures
and zero skips. The initial build caught a GTest helper-name collision, fixed
without changing equations or tolerances. The source identity CTest also passes.
This gate covers the value contract described in the production README, not
whole-interface coefficient transfer or original vehicle attachment admission.

```sh
cmake -S lib_utest/qualification/tied_shell_patch -B BUILD \
  -DCMAKE_BUILD_TYPE=Release -DTL_TIED_PATCH_NATIVE=ON \
  -DTL_TIED_PATCH_CUDA=ON -DCMAKE_CUDA_ARCHITECTURES=120 \
  -DCMAKE_Fortran_COMPILER=EXISTING_GFORTRAN
cmake --build BUILD --parallel 4
ctest --test-dir BUILD --output-on-failure --no-tests=error
```

Owning Bazel host target:
`//lib_utest/qualification/tied_shell_patch:tied_patch_host_test`.
Run builds and GPU qualification under the workspace resource guard and lock.
