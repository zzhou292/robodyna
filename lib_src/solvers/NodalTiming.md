# Nodal timing and the T1 prescribed staggered operation

`FENodalState` remains the only owner of accepted/trial nodal buffers, stream,
epoch, attempt and physical time. `ExplicitNodalStep` supplies non-owning step
operations. The default `VelocityFirst` scheme keeps existing behavior.

`StaggeredHalfKickStart` is currently admitted only by
`AdvanceStaggeredPrescribed` with `NodalStaggeredPrescribedAdmission`: known,
constant, state-independent world forces/couples. It requires rotational nodal
initialization with the existing constant isotropic mass/inertia contract.
Shell, contact, damping and material-history loads need a separate admission.

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
