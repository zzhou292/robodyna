# Selected TYPE25 isotropic response and secondary-row history

This module composes the qualified normal response with the selected native
isotropic friction substage. It is not contact search, source classification,
initial-gap processing, a complete TYPE25 interface, or a vehicle execution path.
Production has no OpenRadioss, Fortran, or GTest runtime dependency.

## Source contract and ownership

OpenRadioss a62b27e6baa555d222a580d6218867d0be4d70b5 supplies the selected
I25FOR3 coefficient and tangential-force blocks, I25IRTLM secondary-row rollover,
and I25MAINF contact-loss reset. Original AGPL notices and the full license are
retained. Native sources and actual control observations are qualification-only.

A fresh reference observation confirms MFROT=2, IFQ=10, IORTHFRIC=0, INCONV=1,
INTTH=0, ALPHA0=1 and normal damping 0.05. VISCFFRIC=0 and FRICC=0.1 were observed
for the first row; this is not proof of an entire population. The selected
no-part-override caller branch sets VISCF=0 and passes it to VISCFFRIC, but the
future source binding must authenticate that route. Unsupported controls reject.
Printed filtering 0 is compatible with raw IFQ10 because Starter selects the
incremental formulation and adds 10. This is a history law, not only exp(mu).

History belongs to the native **secondary row with its retained main association**.
Do not substitute independent generic feature-pair histories. The P2 binding/search
layer must own that association, exclusions, current support and source generation.
These pure packet operations have no authority to establish or change it.

## Response

`EvaluateNativeFriction` consumes explicit normal/friction controls, native-unit
coefficients, current normal and relative velocity, four current main vertices,
post-offset penetration, source stiffness/weights/masses and old/current history.
The arithmetic leaf admits finite supplied normals without a unit-norm tolerance
or renormalization. Native DST3_3 can directly promote a REAL*4 bisector, whose
norm is not exactly one in binary64. A zero vector also follows the native
arithmetic (zero vector resultant with potentially nonzero scalar energy); this
does not establish geometric admissibility. The future geometry/selection stage
owns that obligation. Its normal scalar velocity must match the same dot product of the supplied
normal and relative velocity. The native main diagonal-area expression is used
for coefficient pressure, including the source evaluation when pressure terms
have zero coefficients. No new closest-point or surface-normal method is added.

The normal result comes from the existing P1 kernel. Tangential prediction uses
raw incoming STIF0, not the halved normal-force stiffness or augmented stability
stiffness. It adds STIF0*relative_velocity*DT12 to the old world force, projects
onto the current tangent, and caps its magnitude with the native normal-vector
force norm. The selected source adds no separate quaternion corotation; neither
does this port. Current force history and work are staged only on success.

DT12 is the native tangential increment interval. DT1, carried by normal.dt, is
used for normal history/damping and friction work. They must not be conflated or
inferred from the mid-cycle DT2 estimator. Work is the signed donor increment;
there is no invented positivity clamp. `native_resultant` retains I25FOR3's force
argument sign. The future I25ASS3-compatible adapter supplies positive main and
negative secondary endpoint contributions; this packet does not scatter forces.
Normal stiffness/damping fields are reused without a new friction timestep law.
The selected VISCFFRIC=0 route introduces no extra viscous-friction STI term.

At zero penetration, response history remains unchanged and geometry is not read.
`contact_active=false` marks force/geometry scratch as unobserved in the donor.
The public packet defines those unused channels as zero; comparing those zeros
is not evidence that unassigned native scratch held zero. An old-history field
inside the output can be passed as input: all reads precede publication. Invalid,
unsupported or nonfinite results preserve the complete prior output.

## Native row phases

`BeginNativeHistory` follows I25IRTLM's local-row branch using actual IRTLM fields,
current secondary/main stiffness and the local processor identity. Retained rows
copy current friction force into old force, advance old penetration/stiffness and
clear current slots. Deleted main support, shooting secondary nodes, processor
changes and inactive rows have different source behavior. In particular shooting
clears only markers, and inactive rows clear only PENE_OLD3/4 plus time markers.
PENE_OLD4 is retained as an opaque native payload and PENE_OLD5 as the native
initial-penetration offset. This helper never recomputes either.

`EndNativeContact` is the separate I25MAINF reset after contact classification.
It uses a positive prior IRTLM marker together with TIME_S==EP20 or the native
negative-multiple-of-five marker. It does not replace that predicate with P==0.
TIME_S is retained on this reset. Native row-phase helpers stay in native units;
sentinel values are internal state markers, not new physical timestamps.
The caller still owns valid referenced indices and selected stiffness values.
The test oracle has a bounded 32-entry fixture array; production has no 32-row cap.
No MPI communication or arbitrary source-ID exception is implemented.

## Units and precision

The SI entry converts each dimension to the native working units and back using
P1's shared conversion expressions. C1 scales with pressure^-2, C3 with
pressure^-1, C2/C4/C6 with inverse velocity, and C5/base friction are dimensionless.
For mm/s/tonne the native C6=-0.001s/mm is SI -1s/m. Native speed-squared and
force-squared floors retain their native meanings. A supplied SI scalar VN is
checked against the SI vectors; after converting the components, VN is recomputed
in native dot-product order to avoid a false mismatch from reducing before scaling.

The CMake INTERFACE targets now propagate precise C++ and NVIDIA CUDA flags to
consumers. The qualified toolchains are GNU C++ and nvcc with round-to-nearest,
no fast-math/contraction, precise division/sqrt and no FTZ. Do not override those
usage requirements. Production-only consumer tests inherit them without private
precision options and distinguish fused from unfused arithmetic on host/device.
Bazel users must specify the same compile options; cc_library does not propagate
copts as CMake INTERFACE usage requirements do. Normal-only consumers remain free
of friction/Fortran dependencies.

## Qualification boundary

The native oracle reuses P1's already qualified Fortran normal oracle, then runs
verbatim coefficient/predictor/capping/history/work blocks. Separate verbatim
caller blocks check row phases. Tests cover all returned meaningful fields,
stick/slip/reversal, changing normals, loss/recontact, rollback, SI dimensions,
different DT1/DT12 and actual CUDA execution. P1's owning regressions are rerun
after extracting shared conversion helpers. A pinned actual Yaris positive-entry
input is included, with original sequential velocity interpolation; it is not a
full Engine output comparison. The current source remains unqualified until the
owning execution receipts are recorded. No speed or vehicle-readiness claim is
made by these small response tests.
