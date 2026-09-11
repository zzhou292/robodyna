# Explicit native final-segment continuation

This qualification covers `ShellPlasticityCurveContinuation::NativeLastSegment`.
Existing preparation overloads and source adapters retain `StrictDomain`.
`CurveValue` arithmetic is unchanged: the bounded search already uses the final
segment outside the last knot, with the native strict left-segment convention at
interior knots. The new policy changes admission, not interpolation or the point
iteration. Both accepted and resulting PLA must remain below the native default
EPSGM value (binary32 literal 1e20 promoted to binary64). That separate native
cap branch is not implemented. Unknown policies and non-strict LAW1/analytic
material declarations reject without publication.

Production owns no new history, clock, parser or native runtime dependency.
Catalog `SameScope` includes the explicit declaration; copied/moved catalogs and
resident parameter copies retain it. No homogeneous legacy config, original
source admission or archived schema is changed.

## Source authority

Pin `a62b27e6baa555d222a580d6218867d0be4d70b5` remains unchanged.
Complete SIGEPS44C and VINTER are reused from `../native/law44`, with their
existing source hashes and private-symbol preparation. `NativeCurve.F90` only
packs probes and calls that complete VINTER while retaining its IPOS cache;
it implements no interpolation formula. The point wrapper's added admission
switch defaults to strict and leaves native sentinel constants unchanged.
NativeLastSegment wrapper admission stays below actual source EPSGM, so those
sentinel differences are inactive throughout this gate.

- SIGEPS44C:258–278 evaluates the curve at accepted PLA, then rate-scales yield
  and tangent. Its virgin E tangent and three-iteration update remain unchanged.
- VINTER:83–96 stops advancing at the final segment and extrapolates its line.
- SIGEPS44C:325–335 has a distinct EPSGM cap branch; HM_READ_MAT44:191–194,228–285
  defines the default. This qualification intentionally excludes reaching it.
- Complete conversion/starter bytes for DEFINE_CURVE and FUNCT are durably
  authenticated in the workspace `openradioss-shell-hardening-continuation-1`
  source manifests. They preserve original point counts and identity transforms.
- `source-curves.json` retains original key identity, source line, byte count and
  SHA256 for each exact card. `verify_curves.py` checks the cards and reproduces
  the binary64 SI arrays in `OriginalCurves.h`; it performs no runtime admission.

The original FAIL=1 tables are LCID2100180 (17 points, endpoint .5), LCID2100220
(15, .4), LCID2100271 (11, .3). Stress conversion is original MPa times1e6.
Their explicit positive-rate control is C8000/s, P8, resolved cutoff10000/s;
rate-off is an additional mathematical control, not the original declaration.
Tests apply controlled increments to original tables; they are not vehicle
trajectories or a statement of full source material/case admission.

## Tests and arithmetic scope

Four host value tests cover all original endpoints, exact before/after probes,
strict-default rejection/crossing/retry, unsupported cap/invalid declarations,
finite-input overflow and FAIL=1 partial/removal/post-inactive recurrence.
Two catalog tests add full policy identity/copy/move/rebind and late invalid
LAW1/analytic/policy rejection; all14 previous catalog tests are retained.

Three native tests compare cached VINTER knots/continuation, every point result
through crossing/loading/unloading, and complete NIP3 failure caller histories
for all8 initial failed-point masks, all3 original curves and rate on/off. Mask0
starts from virgin source-shaped history and crosses the original endpoint before
FAIL=1; other masks start beyond the endpoint to qualify old-FOFF behavior.
Every sequence stops after two already-inactive intervals. Native state evolves
independently, including its own thickness, saved stresses, rate and failuretime.
Existing `FailureNativeChecks` compares current force stress, saved masked point
stress, resultants, tangent, thickness and signed work; no new tolerance is used.

Two optional CUDA functions prepare parameters on device and advance their own
point/section histories. They compare the same complete native value oracle,
including all8 masks and post-removal section/work paths. Rejected point updates
preserve device history/result bytes, then retry. A third-point cap failure
preserves the entire section output and device history before the same trajectory
retries without host reseeding. Section viscosity is supplied
as the independent native caller's returned coefficient, as in the existing
failure gate: this qualifies work arithmetic, not a new DM coefficient formula.
No resident owner, contact deletion or full vehicle dynamics is claimed.

Author host checks use one CPU/512MiB. Native/Fortran/CUDA compilation and
execution are root-owned qualification gates; host syntax of CUDA-shaped code
is not device qualification.

## Owning gates

```
cmake -S lib_utest/qualification/shell_hardening_continuation -B BUILD \
  -DCMAKE_BUILD_TYPE=Release \
  -DTL_HARDENING_CONTINUATION_NATIVE=ON \
  -DTL_HARDENING_CONTINUATION_CUDA=ON
cmake --build BUILD --parallel 1 --target \
  shell_hardening_continuation_values_check shell_plasticity_binding_check \
  shell_hardening_continuation_native_check shell_hardening_continuation_cuda_check \
  shell_layered_failure_values_check shell_layered_failure_native_check
ctest --test-dir BUILD --output-on-failure \
  -R '^(shell_hardening_continuation_|shell_plasticity_binding_check$|shell_layered_failure_)'
```

CMake also retains the existing host collection and constant-failure targets;
only run targets that were built. Root should retain the previous standalone
LAW44 point/rate/analytic native gate when qualifying the shared wrapper.
Bazel owns the pure value target and catalog tests; no native oracle enters a
production target. Complete native source verifiers run through existing target
dependencies; no source hash or donor revision is changed by wrapper wiring.
