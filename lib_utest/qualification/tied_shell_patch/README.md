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

The subsequent coefficient extension passes six additional functions (three
host, one independent native, two actual CUDA), with all nine original functions
passing again. Exact explicit I2FOR28_CIN442–503 and independently computed
geometry cover four shapes and eight input modes. Current master IN is zero in
the reference packet while supplied MINER can be positive; repeated triangles
use J4=J3 and both native additions target the same physical node. This checks
both inertia-transfer branches, zero physical coefficients, native DMAST versus
source mass and exact zero/EM20 dependent coefficients. Invalid final input and
computed overflow preserve the complete previously published GPU value packet.

Evidence: `tied-coefficients-root-functions-1` contains all15 passing functions
with zero failures/skips; guards are `tied-coefficients-root-{build,tests}-*.json`.
The first build caught two GTest trace declarations on one line, fixed without
equation/tolerance changes. Independent code review found no blocker. This adds
the coefficient value stage, not source classification, retained release history,
source-ordered multi-slave assembly or a connected original vehicle owner.

`NodalRepeatedForceAssembly` closes the four-slot triangle scatter boundary by
reusing the strict existing single-node operation on private scratch. It retains
the native order of additions, including a cancellation control for which
precombining repeated-slot loads gives the wrong result. The legacy distinct-node
API remains unchanged and continues to reject repeats. Five additional functions
(three host, one exact-native accumulation, one CUDA) pass alongside all15 earlier
functions in `tied-repeated-assembly-root-functions-1`, with zero failures/skips.
Native I2FOR28_CIN426–437 independently accumulates the native loads for four
shapes and16 packets, including a real repeated triangle. Final connectivity,
nonfinite input and overflow failures leave all destination components unchanged.
The helper does not authenticate topology or a nodal-owner transaction.

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
