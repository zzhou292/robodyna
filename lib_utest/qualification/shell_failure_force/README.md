# Failure-aware native shell force values

This increment adds explicitly named QEPH/T3 `LayeredJ2FailureHistory`,
`LayeredJ2FailureForceTrial`, `InitializeLayeredJ2FailureHistory` and
`EvaluateLayeredJ2FailureForce`. It composes the qualified centered NIP3 local
constant-D1 section caller with the existing native family force operation.
The trial contains the complete ordinary `ForceTrial` and complete section
failure result. Saved point stress, current force stress, damage/time/activity,
material resultants, plastic increments and signed work remain distinct.

The caller explicitly accepts shell and section values together. There is no
resident collection, state owner, clock, source-material admission, mass removal
or contact-surface removal in this API. `PrepareFailurePrescribedHistory` admits
finite prescribed accepted OFF0/1 values; it authenticates exact reference input
association, not provenance or a live owner. Legacy `PreparePrescribedHistory`,
`EvaluateForce` and `EvaluateLayeredJ2Force` remain active1-only. Pending OFF0.8
is rejected. Failure preserves all input/output objects; no partial trial escapes.

## One force path per family

Each previous layered force body is moved to its family's `LayeredForceCore.h`.
Small shared section adapters select the legacy or failure caller. Family
geometry, viscosity, work, QEPH plastic stabilization and projection keep their
original operation order. Legacy resultant admission is unchanged; failure
admission checks **current force-point** resultants against shell material fields
and separately checks exact parent activity. It never derives force from the
masked saved point stress.

Pinned OpenRadioss revision: `a62b27e6baa555d222a580d6218867d0be4d70b5`.

| Donor | Explicit failure behavior retained |
|---|---|
| `czfintn.F:209,356–366` | Final OFF enters old stabilization work C5, force factor C8 and viscous HVL. Elastic increments and NPT3 plastic correction still execute OFF0. |
| `cndt3.F:137–158` | OFF0 assigns STI/STIR zero; the separately computed DTEL remains positive. |
| `c3dt3.F:122–135,210–219` | Its own selected NODADT branch also assigns zero stiffness and computes DTEL; no QEPH formula is substituted. |
| `cupdtn3.F:85–92` | OFF<1 publishes OFFG, but only negative OFFG triggers its additional force suppression. No negative-OFF or global scatter policy is introduced. |
| `czdef.F:175–195`, `c3coor3.F:147–158` | OFFG0 retains rates; the negative flag has a different branch. |

Inactive geometry must still satisfy the existing finite nondegenerate family
domain. A collapsed/singular removed parent rejects the value trial. This is an
explicit qualification boundary, not a blanket freeze or a claim to support all
native degenerate-geometry lifecycle branches. Native M/J stays unchanged.
Zero stiffness is allowed only alongside an explicit accepted inactive parent;
legacy diagnostics still require positive stiffness and all paths require a
positive finite DTEL.

## Independent native and CUDA gates

`native/prepare_sources.py` hashes thirteen existing native adapters/manifests and
donors, then generates four checked adapters from the existing complete layered
family control drivers. It reuses their coefficient preparation and complete
geometry/strain/stiffness/force/projection leaves. The material call changes to
the existing `LF_CALLER::layered_failure_caller_ssp`, which owns complete LAW44 and
Johnson leaves plus exact MULAWC/FAIL_SETOFF_C extracts. The bridge passes the actual coefficient producer's `M%SSP(1)` into the
shared caller. The standalone caller retains its original plane-stress speed
and unchanged C ABI; its reconstructed speed cannot substitute for the family's
prepared coefficient. The first root gate exposed this adapter error as a
4.8% viscosity inflation at nu=0.3, before removal; its failing reports remain
recorded. An explicit native coefficient-association test checks this seam for
both families, including inactive intervals. There are no production
equations in this native bridge and no second copy of the donor libraries.

The T3 geometry/rate wrapper has an active1-only **wrapper** guard after its
OFF-independent C3DEFO3/C3CURV3 leaves. The new adapter runs that unchanged
nondegenerate geometry wrapper then restores the actual history OFF before the
section/stiffness operations. C3COEF3 declares but never reads its OFF dummy.
This explicit oracle context adaptation is included in the authenticated
generated text; the old native API is unchanged. Native section time is the
actual prescribed endpoint, not midpoint or accepted base time.

Native packets retain their own shell/point/failure histories between calls.
Six native tests cover all eight initial failure masks, table rate off/on,
analytic positive-C/P rate on, final removal and two subsequent intervals with
nonzero deformation, viscosity and QEPH hourglass rates. Three original
positive-FAIL curve tables also exercise their independently qualified last-
segment continuation from explicitly prescribed beyond-table histories.
Analytic rate-off remains outside its existing preparation contract. Tests
compare complete native geometry, force/couple, history/work, current/saved
section fields, exact failure timestamps and zero stiffness using existing
comparison budgets. These are prescribed trajectories, not original impact runs.

Two CUDA tests keep shell and failure sidecar histories in device memory,
exercise the same table/rate/mask/removal sequence, inject a last-point invalid
history, verify untouched accepted/output bytes, then compare every named trial
field against a separate clean **device** retry. Native comparisons remain
independent. No host/device bit-equality assumption substitutes for that retry.

Standalone CMake owner: this directory. Host target/CTest:
`shell_failure_force_values_check` / `shell_failure_force_values`.
Enable `TL_FAILURE_FORCE_NATIVE=ON` for `shell_failure_force_native_check`;
enable `TL_FAILURE_FORCE_CUDA=ON` for `shell_failure_force_cuda_check` (also
enables native prerequisites). The two matching CTests are
`shell_failure_force_native` and `shell_failure_force_cuda`. CPU-only author
checks do not constitute Fortran or CUDA execution evidence; the parent owns
those serialized gates and the affected existing resident regressions.

## Root qualification

At TL `1017522`, all **13 new functions pass**: four values, seven independent
native checks and two actual CUDA device-history tests. All **36 affected
regressions** also pass: four legacy failure-caller native functions, 18 mixed
layout/resident functions, five legacy native force recurrence functions, one
legacy layered CUDA function, and eight original-source app CUDA functions.
No skips or comparison-budget changes were used. The latter app gate builds
current app `aa2f3ce` against the same main TL headers and includes the original
149-shell elastic and 631-shell mixed fixtures. The owning Bazel value target
also builds (`shell-failure-force-bazel-build-1`).

Reports in the workspace are `shell-failure-force-root-tests-2`,
`mixed-layered-resident-root-regression-tests-1`,
`shell-failure-force-legacy-regression-tests-1`, and
`source-layered-flight-root-regression-tests-1`, with corresponding per-function
XML directories. Preserve the initial failed runtime gate: the complete family
driver needed to supply native M%SSP explicitly to the test bridge. The production
force equations were unchanged. The initial CUDA fixture compile failure was
fixed by placing its two gtest trace macros on distinct source lines.

This qualification covers force values and existing owner regressions. Optional
resident failure history, source admission, contact activity and full-shell
impact remain separate integration work.
