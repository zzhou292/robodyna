# Local LAW44 / constant-D1 NIP3 caller

This qualifies section values only. It adds no collection storage, accepted
selector, owner, contact removal, source admission or full-vehicle execution.
Native Fortran/runtime qualification is pending the parent's guarded gate.

The admitted source subset is centered NIP3, one layer, one in-plane point,
local isotropic LAW44 with existing analytic or tabulated hardening, positive
constant Johnson D1, IFAIL_SH=2 and the missing/zero property thickness criterion.
The original source has no MAT_ADD_EROSION override. Starter PTHK becomes
ONE−EM06; WF={.25,.5,.25}, while WM retains the separate native float literals.
Only all three failed points reach this positive threshold. Numeric OFF=0.8 is
the native pending-removal marker, consumed before the completed result appears.

`ShellLayeredJ2FailureHistory::saved` is the next constitutive input. Its five
stresses are masked for failed points, but PLA/rate retain native updates.
`current_force_point` records the unmasked current stresses used in resultants.
The result's `current.history` also contains unmasked current point results;
it must not be substituted for the next saved history. A partially failed
section can therefore have zero saved stress and nonzero current force from
the same point. After parent removal, rate/predictor updates still run, plastic
gather stops and local thickness increments vanish; complete force/contact
activity propagation remains a later gate.

The shared point recurrence now accepts trailing `element_active=true` by
default; only native OFF-dependent gather/thickness branches change when false.
The shared section loop has a const point-result observer and retains the exact
legacy reduction order. Legacy APIs and saved-history/resultant checks are not
weakened; the new failure-aware match uses the actual current-force history.
Generalized work adds viscosity before the final parent mask and retains the
old-force half of removal-interval work. Its wrapper stages failure atomically.
The work adapter reconstructs raw current resultants with the existing NIP3
reduction before adding viscosity and applying OFF; the separately exposed
material-only values remain parent masked. Applying viscosity to already-masked
material values changes signed zero and can conceal overflow of the raw sum.
Focused value cases check both, full work preservation and exact retry. A native
OFF0 biaxial predictor case compares zero signs against the unchanged caller
extract, including a negative control for the wrong order.

Native MULAWC:2068 overwrites local failure DPLA with PLA(new)−PLA(old).
MULAWC:2011 uses that same rounded subtraction for its plastic-work diagnostic.
The shared diagnostic was corrected accordingly; `plastic_increment` still
contains the distinct constitutive iterate. The sub-ULP test deliberately makes
that iterate positive while caller DPLA, work and damage increment are zero.
This is a documented numerical correction to diagnostics, not an archive rewrite.

The native oracle uses complete unchanged SIGEPS44C and FAIL_JOHNSON_C donors
through their existing owned native libraries. The point bridge takes explicit
parent OFF and evaluation TT as optional arguments; old calls retain defaults.
`native/source-manifest.json` authenticates three complete callers and 19 exact
selected fragments. NativeParent retains the starter/default and positive
criterion operations; NativeCaller retains stress saving, work, accumulation
and parent publication operations. The retained native buffers are independent
of production results. Native contact and whole-engine deletion are not mocked.

Tests cover every failed subset, table rate on/off and admitted analytic VP2,
all saved/current
stress fields, PLA/rate/tangent/yield/thickness, partial failure, final removal,
two post-removal evaluations, native/generalized work, sub-ULP rounding, late
third-point rejection, unchanged output and successful retry. First host pass:
six value functions; native C++ sources syntax checked only. Fortran is not run
under the independent author resource allowance.

The source curve endpoint remains a separate admission obligation. Original
FAIL=1 materials use LCID2100180 (ends at PLA=.5; MIDs2000173/2000319), LCID2100271
(ends at .3; five MIDs including2000190/2000322) and LCID2100220 (ends at .4;
MIDs2000191/2000352). All three endpoints are below source FAIL=1. Existing
HardeningDomain still rejects an out-of-domain update, before a failure callback
can remove the point. This increment does not invent extrapolation, clamp PLA,
raise limits or claim that a full source failure event is reachable. The native
caller tests use explicitly declared synthetic table/analytic material inputs.

Host gate (one CPU):

```
cmake -S lib_utest/qualification/shell_layered_failure -B <host-build> -DCMAKE_BUILD_TYPE=Release
cmake --build <host-build> --parallel 1
ctest --test-dir <host-build> --parallel 1 --output-on-failure
```

Parent native gate adds `-DTL_LAYERED_FAILURE_NATIVE=ON`, then builds
`shell_layered_failure_values_check` and `shell_layered_failure_native_check`.
The existing native point/rate/analytic, layered Q/T recurrence and default
plasticity CUDA regressions remain the affected qualification gates.
