# TYPE45 supplied-context joint values

This module implements automatic-stiffness spherical, revolute and cylindrical
TYPE45 joints with scalar free-DOF coefficients. It reuses TL Fixed3 geometry
operations and the pinned native RINI45, RINI45_RB, JOINT_BLOCK_STIFFNESS,
RSKEW33, RUSER33, RDTIME33 and RCUM33 equations. It adds no participant,
source-property factory, topology registration, owner commit or time-step selector.

`Reference::Prepare` owns and validates the supplied source EID/NIDs, original
geometry, resolved property and TT0 coefficient context. Structural endpoints
use the same M/J and position for damping and automatic K. A rigid member instead
has independently supplied body mass/mean-principal J for damping, and main-node
M/J/K/Krot plus its main coordinate for automatic stiffness. A rigid main node
is rejected. The caller must authenticate actual registration and the native
startup phase; this value object does not provide that proof.

The property is already resolved: Kn is zero (automatic), ScF is positive and
Cr is in (0,1]. Free K/C values must be canonical zero on blocked DOFs. Native
critical damping acts only on blocked DOFs, while explicit free K/C remain
distinct. The native maximum damping diagnostic uses the maximum Cr across
all six DOFs, including when its maximum stiffness comes from a free DOF.
No curves, stops, friction, sensors, external skews or added mass are admitted.

`startup()` exposes RINI45's pre-automatic stiffness and viscosity observations.
`automatic_stiffness()` exposes resolved K and the native structural floor flag.
These stages do not create an assembled TT0 force cache. `History::Initialize`
creates only the virgin frame/history. `Evaluate` takes a positive supplied
interval, authenticates exact reference values and caller time/sample continuity,
and stages the complete next history, endpoint forces/couples, signed work and
nodal stiffness. The caller decides whether to retain that result. Failure leaves
the entire output unchanged, including when accepted history is part of output.

All public values use SI. `WorkingUnits` is closed to SI and millimetre/tonne/
second; it determines the conversion of native length, squared-length, M/J and
stiffness thresholds. Current SI coordinates are never converted out and back
inside production. Source default-REAL literals 4.1, 0.8, 1.3 and 1e-8 retain
their binary32-to-binary64 values. Unit-equivalent evaluation has ordinary
binary64 reduction roundoff; no global native bitwise claim is made.

Original source options remain separate evidence. All 44 original regular joint
RPS/DAMP columns are blank; the converter and SDI export/default paths must be
resolved before an automatic app property factory is admitted. The original
geometry qualifier uses explicit supplied properties/body context and retains
all N1–N5. It does not turn that fixture into source mechanics or owner evidence.

The complete source oracle and tests are in
`lib_utest/qualification/type45_joint`. The source pin is
`a62b27e6baa555d222a580d6218867d0be4d70b5`; its manifest records complete donor
bytes, SHA256 and Git blob identities. Production value and native/GPU execution
evidence must be reported separately until the owning gate passes.
