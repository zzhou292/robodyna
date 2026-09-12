# Complete selected-model free flight

This case adapter advances the existing physical owner and six mapped
participants. It does not create a solver, second clock or source coefficients.
Actual accepted Q/T/QBAT activity feeds the existing CIN witness adapter before
the native tied/rigid step. All material candidates and complete field readback
finish before the common publication. Discard/retry preserves the accepted
selection. Step and capture workspace is bounded before factory allocation.

The default profile applies no external force or wall contact and contains no
TYPE45 contributor. It is a complete-count integration diagnostic for the
selected physical model, not a connected-vehicle crash. Uniform-motion errors
are observed from actual computed positions, velocities, orientations and spins
against original coordinates; the adapter never sets prescribed trajectories.

Rejected solid and structural-beam stages throw `NativeStageError`, preserving
the report's typed batch status, reported solid family, parent/node indices,
native element status and nodal status. The exception owns its message and copies
only scalar context; it borrows no report, source or device storage. The concise
stage message includes these values, so the existing run loop retains them in a
failed accepted-prefix reason. A solid parent index addresses its reported family
span; a beam parent index addresses the beam span. Neither index is an EID/PID.
`SIZE_MAX` is reported as unavailable, including reports that identify no parent.
The native integer element status keeps its family-specific meaning (including
the existing `-1` cache-validation sentinel); no failure cause is inferred from it.
Other report types retain the existing generic exception behavior.

Success checks, complete discard on a failed attempt, and the common commit path
are unchanged. No success-path array, readback, source lookup or clock is added.
The standalone CXX-only `tests/reports` gate checks report ownership, exact typed
conversion and late solid/beam failures through the existing run-loop test seam.
Its prefix tests qualify host orchestration, not material numerics or GPU rollback.

The public preparation/commit/discard phases allow the caller to inspect the
candidate before accepting it. An explicit optional `JointModel` now authenticates
and adds the seventh TYPE45 participant through the same startup/common publisher.
The separate [`LoadedWall`](../vehicle_wall/loaded/README.md) factory installs one
authenticated finite-wall stage before sealing and after material preparation.
The default null-joint/no-wall path is unchanged. Accepted material fields are already
owned by TL; the separate capture module will serialize their real histories.

Qualification passes two small motion-observation checks, a complete original
forecast and four actual free-flight intervals including an unpublished trial
followed by exact retry. Reports are `vehicle-physical-step-values-tests-2`
and `vehicle-physical-free-flight-tests-1`. Complete host workspace is
77,847,455 B; inclusive host bound 7,364,211,510 B. Actual sampled RSS is
3,423,043,584 B and device growth 4,523,556,864 B under the 6 GiB growth guard.
The preserved first forecast failure came from its CUDA search running without
device access; the second owning forecast passes.

At the reserved 1e-8 s interval, four accepted intervals end at 4e-8 s.
Maximum position error is 1.11e-16 m and velocity error 1.94e-11 m/s.
This short diagnostic is not a visible-motion video. The 170.155 s complete test
(startup plus five attempts) also establishes that the current serial assembly
path is not yet practical for a 20 ms full-vehicle run. Physical step selection
and bounded parallel assembly remain separate required work for that demo.

## Optional physical timestep screen

`Config::structural` opts into TL's post-CIN screen with an explicit factor.
The ordinary native mass/stiffness formula and current rigid-body analytical
trace remain owned by TL. `StepObservation::structural_step_limit` preserves
its actual returned minimum for an accepted candidate; zero means disabled.
A `StepSizeError` carries the rejected limit and physical node after the usual
complete discard. It does not change the fixed timestep in an existing owner;
a new configured run is needed to select a different step.

The `vehicle_physical_dynamics_structural_screen` original-model gate uses
factor 0.8 at the existing 1e-8 s diagnostic step. It checks every selected
operator's free motion, commits once, records the actual minimum, and verifies
unchanged allocation counts. This local structural screen is distinct from
retained-joint completion or a global nonlinear crash stability guarantee.

`StepObservation::wall.enabled` distinguishes an installed wall from unavailable
contact data. Accepted and prepared wall observations come from the actual owner
attempt; only `last_accepted_step()` is an accepted interval record. Existing
`uniform_motion` fields remain measured deviations from the declared free-motion
baseline, including on loaded runs; they never prescribe the trajectory.

## Optional call timing

`Config::timing.enabled` opts into the existing bounded `StageTimer`; the default
is disabled. `timing()` returns copied diagnostic counters. Named stages separate
each accepted family/joint/wall assembly, the CIN/rigid advance, each candidate,
prepared field capture, commit and discard. Slot zero is the inclusive
`PrepareStep` call. Other named calls remain in their original order. Rejected
calls contribute failed-stage counters, while accepted mechanics are unchanged.
No extra CUDA call or synchronization is added. Host wall time can include waits
already performed by each operation; it is not a device-kernel duration.

The timer is inline in the existing storage and charged by its existing
`sizeof(Storage)` forecast for both enabled and disabled configurations. It has
no per-step allocation. The previously qualified timer handles invalid/backward
clock reads and saturates counters without changing a mechanics result or
exception. This is independent of the sole physical timestep/owner epoch.

The original loaded two-interval/discard-retry test explicitly enables timing
and records `stage_ns_*`/`stage_calls_*` properties. It expects three prepared
attempts and two common commits while retaining all prior numerical/source
checks. Root owns this actual gate. Author validation reuses the standalone
timer host tests and complete physical production/test syntax checks.
