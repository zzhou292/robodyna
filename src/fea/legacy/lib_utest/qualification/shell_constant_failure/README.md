# Constant plastic-strain point failure

This leaf implements the source converter's constant Johnson-Cook failure
branch: D1 is the supplied positive failure strain; D2 through D5 and EPSF_MIN
are zero. It accumulates the actual positive plastic-strain increments while
both the element and point are active, records the native failure evaluation
time at damage >=1, then saturates damage. An inactive point retains its first
failure time. Elastic unloading does not reverse damage. No fracture energy,
stress reduction, element deletion or contact removal is inferred here.

The independent oracle compiles the complete pinned FAIL_JOHNSON_C routine.
Preparation verifies exact original bytes and Git blobs, renames the leaf and
removes its unused CRACKXFEM_MOD import. Existing qualified LAW44 native build
infrastructure supplies precision, constants and includes. Logging uses the
native routine's ordinary output units. No production C++ failure arithmetic
is called by the oracle. Native TDEL is read only on a new point failure; its
INTENT(OUT) contract leaves other entries undefined. Persistent timestamps are
carried explicitly from caller history on every other branch. The initial point branch excludes rate, temperature,
triaxiality-dependent fracture and element-level IFAIL_SH behavior.

The source converter emits this as a separate /FAIL/JOHNSON model and clears
LAW44's own EPS_MAX cutoff. The staged starter and FAIL_SETOFF_C donors are
reference evidence only. Before source/runtime admission, independently qualify
their cross-point removal rule, caller evaluation time, stress/history and work
phases, native element activity and contact lifecycle. A point failure flag does
not itself establish an eroded parent. This feature adds no owner or clock.

Owning qualification:

```sh
cmake -S lib_utest/qualification/shell_constant_failure -B build/shell-constant-failure -DCMAKE_BUILD_TYPE=Release -DTL_CONSTANT_FAILURE_CUDA=ON -DCMAKE_CUDA_ARCHITECTURES=120
cmake --build build/shell-constant-failure --parallel 4
ctest --test-dir build/shell-constant-failure --parallel 1 --output-on-failure
```

Three value, three native and one actual CUDA test pass in the guarded owning
run `crash-work/reports/shell-constant-failure-tests-2.{json,xml}`. They cover
accumulated loading, hold/unloading, exact threshold, inactive flags, persistent
first failure time, positive-overflow saturation and unchanged rejected output.
The initial build exposed a missing declaration for the shared finite-value
helper; its owning header/dependency was corrected, with no arithmetic or
tolerance change. Source and coupled-shell runtime admission remain pending.
