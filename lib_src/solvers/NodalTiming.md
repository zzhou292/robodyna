# Explicit nodal timing and restricted admissions

`FENodalState` remains the only owner of accepted/trial nodal buffers, stream,
epoch, attempt and physical time. `ExplicitNodalStep` supplies non-owning step
operations. The default `VelocityFirst` scheme keeps existing behavior.

The original `StaggeredHalfKickStart` operation is
`AdvanceStaggeredPrescribed` with `NodalStaggeredPrescribedAdmission`: known,
constant, state-independent world forces/couples. It requires rotational nodal
initialization with the existing constant isotropic mass/inertia contract.
Shell, contact, damping and material-history loads require the separate
restricted admission described below and their own case qualification.

Initial x/q/v/omega are collocated at time zero. The first accepted trial kicks
v/omega by h/2, then drifts x/q by h. Later trials kick and drift by h. A rejected
first trial does not consume the half kick. Stored velocities thereafter are
previous midpoints; `velocity_time` is the represented `base_time+h/2`, retained
exactly, while positions/orientations have endpoint time `base_time+h`.
The world quaternion increment uses the same existing rotation utility.

The stamp and borrowed views explicitly carry scheme and velocity phase.
Reaction forces/couples still refer to the consumed base load. The separate
`reaction_kick_dt` identifies its momentum-kick duration; it differs from the
physical interval h on startup. Kick energy and drift work are distinct ledgers.
Commit publishes all timing fields only after the normal CUDA error checks
and stream completion. Discard and failed commit preserve the accepted stamp.

There is no added device allocation, state slab, history cache or second clock.
The owner remains bounded to 64 nodes, two state slabs and six allocations.
Legacy shell/contact/output consumers reject the new scheme, including its
collocated epoch zero, until their own sampling/output contracts are extended.
Existing archive schemas therefore keep their original interpretation.

T1 passes seven new owner CUDA functions and one new actual-output function.
The affected owner/contact/case/artifact regression set passes 146 functions;
owning Bazel temporal and rotation targets also pass. Independent review added
unsupported-timing tests to two existing artifact functions, both passing.
Evidence is under workspace `crash-work/reports/nodal-t1-*`.

The numerical tests cover exact constant-force quadratic positions, independent
constant-torque world rotations, kick energy versus drift work/reactions,
scheme misuse, finite overflow and retry, prepared discard, a safe invalid CUDA
launch before commit, midpoint rounding and unchanged allocation capacity.
They do not establish native donor startup, variable steps, elastic stability,
nonzero initial stress, joint material-history commit or vehicle dynamics.

The T2 `AdvanceStaggeredHistory` source now accepts a distinct
`NodalStaggeredHistoryAdmission`: owner/epoch/attempt, maximum dt and angle,
and a nonzero case qualification identity. It always requires the existing
matching `NodalValidationReceipt` before Commit. It leaves the old velocity-first
elastic-rate formula unchanged and does not apply it to a history recurrence.
The case must qualify its full state/history/velocity recurrence and candidate
envelope; native element dt and a force-only tangent are not that argument.
External material publication remains the coordinator's responsibility.

Before first execution, five new CUDA functions freeze the existing `2e-13`
SI arithmetic tolerance. The independent scalar oscillator uses m=2 kg,
k=8 N/m, constant F=2 N, h=1/8 s and a physical-rest half kick. Its exact
discrete response is x_n=(F/k)(1-cos(n theta)), with
cos(theta)=1-h²k/(2m); h*sqrt(k/m)=1/4 is inside its stability interval (0,2).
Thirty-two steps compare x and carried v against this wider-arithmetic formula.
Other functions cover missing/foreign receipts, unchanged accepted bytes and
startup kick after rejection, identity/limit misuse, legacy-policy rejection,
and a safe invalid CUDA launch during validation. The seven T1 test bodies are
unchanged; their common fixture helpers now live in one qualification header.
All five T2 functions now pass on the actual GPU, alongside all 36 affected
T1/rotation/legacy-step functions. Owning Bazel history-admission, temporal and
rotation targets pass. The first fault fixture incorrectly required error 9
while this runtime returned error 1 for the zero-block launch. The corrected
fixture uses the already-qualified T1 contract accepting either code, with a
clean-error precondition; no solver arithmetic or numerical tolerance changed.
The first execution remains in `nodal-t2-first-execution-1`. Corrected runtime
reports are `nodal-t2-fixture-{build,tests}-1.json`, the initial regression XML
is `nodal-t2-owner-xml-1`, and `nodal-t2-bazel-1.json` records the owning builds.
This gate does not qualify a QEPH coupon or joint material publication.
