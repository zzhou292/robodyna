# Rigid-group kinetic and kick-work values

`ObserveGroupKinetic` and `ObserveGroupKick` are bounded host/device value
functions. They own no state, clock, CUDA storage or source binding. They do not
advance a group, produce another mass/inertia model, perform an eigen solve,
publish a history or add material work. The caller supplies the authenticated
owner/model association. Every failure leaves the output value unchanged.

`NodalRigidObservationTypes.h` defines the inputs and channels;
`NodalRigidObservationMath.h` supplies shared finite/metric/phase checks and
compensated sums. `NodalRigidKineticObservation.h` evaluates individual samples;
`NodalRigidKickObservation.h` checks the actual stored-velocity kick.

## Explicit stored phases

Each sample requires `ObservationPhase` with a named kind and position,
velocity and principal-frame times. The admitted kinds are:

- `PhysicalInitialization`: all three times coincide.
- `StoredMidpointWithLaggedFrame`: endpoint position, preceding midpoint
  velocity, and lagged force-stage principal axes. The midpoint lies between
  the supplied frame and endpoint times, including binary64 time rounding.

The second kind defines a phase-specific diagnostic metric. It does **not**
assert spatially collocated physical kinetic energy or native `RGBCOR` output.
The input axes are used directly; neither endpoint shell quaternions nor a
freshly extrapolated frame may be substituted silently. A kick requires the
next lagged frame time to equal the previous position time, and the supplied
kick duration to match the change in velocity times within absolute-time
rounding. These are value checks, not another accepted-time authority.

## Metrics and exactly-once partitions

`GroupObservationMetric` supplies the existing prepared group properties and
its complete group-local member span, in source order. At most 256 members are
admitted. Native scalar J is authoritative; physical/added partitions are
checked at the existing 64-epsilon consistency tolerance. Group sums, source
correction and the retained reference tensor/principal representation are
checked without recomputing eigenvectors or changing any inertia.

The raw physical-node diagnostic is:

```
K_members = sum_i (m_i |v_i|^2 + J_native_i |omega_i|^2)/2
```

The group diagnostic is evaluated directly from group translation and its
full world tensor, represented by the supplied axes and corrected principal J:

```
K_group = (M_group |v_group|^2 + omega_group^T R J R^T omega_group)/2
replacement = K_group - K_members
```

Reference member arms are expressed in the existing reference principal basis
to report the corresponding group orbital rotation. The rotational decomposition
is member orbital energy, native member scalar-J energy, primary parallel-axis
energy, primary isotropic-J energy and source principal correction, each once.
Physical and added member-J energies are supplementary partitions of the native
scalar-J term; neither is added again to the group total. Primary translation
is separately reported as part of the aggregate's existing total mass.

The decomposition has an explicit numerical budget for the existing 1e-12
principal-frame admission, source-order startup reduction and observation
arithmetic. The actual corrected principal metric remains authoritative. The
native-minus-physical-minus-added member residual remains visible rather than
replacing the native J by a rounded partition sum.

A whole-collection consumer should accumulate free-node K and effective group
K directly. It may also report raw free-node plus raw grouped-member K. It need
not subtract small groups from a potentially enormous all-node kinetic sum.
These value functions do not infer free-node membership or authenticate a
caller-constructed group span.

## Actual kick identity and roundoff

Applied and constraint-reaction force/couple work are separate channels. For
each scalar physical member DOF with metric c = m or native J:

```
average = v_before/2 + v_after/2
delta_K = c (v_after - v_before) average
W_applied = kick_dt F_applied average
W_reaction = kick_dt F_reaction average
```

Each impulse is independently checked before global reduction. This catches
corrupt opposite contributions and zero-average reversals that an energy-only
sum can miss. Its binary64 budget is:

```
64 epsilon [ c (|v_before| + |v_after|)
             + |kick_dt F_applied| + |kick_dt F_reaction| ]
```

The endpoint-magnitude term admits rounding of a kick smaller than one stored
velocity ULP; a work-only relative tolerance would incorrectly reject it. The
constant bounds the short native/TL kick, reciprocal, reaction and observation
arithmetic, not model error. Positive work-error budgets additionally cover the
average-velocity products and compensated reduction. Final scalar combination
rounding is included explicitly. Tests require materially corrupted reactions
to fail at the corresponding member/DOF.

Group delta K is also evaluated as stable quadratic differences in group
translation and local principal-spin components. This includes the change in
the explicitly supplied frame phase. Replacement delta is formed from the
group and native-member **deltas**, avoiding subtraction of four large stored
kinetic totals. The output separately checks:

```
delta K_native = W_applied + W_reaction
delta K_group = W_applied + W_reaction + delta replacement
```

Because replacement delta is defined as aggregate delta minus native delta,
the effective residual is the same native kick identity plus floating-point
bookkeeping. It is not an independent check of aggregate dynamics.

Reaction work and replacement delta may have either sign. They are not material
or artificial dissipation, and a small algebraic residual is not evidence of
accurate constrained response. No elastic/plastic/stabilization work is added by
this module. `KickMismatch` includes the offending DOF or group residual and
budget; no partially computed observation is published.

## Native collocated observation remains separate

See `NodalRigidCollocatedObservationDesign.md` for the pinned force-stage call
order and proposed input. No collocated phase is implemented here. The native
kinetic wrapper fixes `DT1=0` and A/AR=0 around exact correction fragments to
qualify supplied-value arithmetic. This is explicitly not native collocated
observation, native engine execution or an owner force-stage reconstruction.

## Qualification and build targets

The independent scalar long-double oracle constructs the full world tensor
from member arms, native scalar J and primary/correction ledgers. Cases include
dense anisotropy, observable primary mass/COM/J, source-corrected inertia, world
rotation, full force and couple work, 64 changing/off-load kicks, native 32-step
reactions, corrupted final-member reactions, zero-average reversal, sub-ULP
kicks, rounded absolute times and overflow rollback. A small CUDA fixture checks
the same value functions and unchanged failure outputs.

CMake include: `NodalRigidObservation.cmake`. Qualification targets are
`nodal_rigid_observation_check`, `nodal_rigid_observation_native_check` and,
with `TL_NODAL_RIGID_CUDA_CHECKS=ON`, `nodal_rigid_observation_cuda_check`.
Corresponding production/host/CUDA Bazel targets use `nodal_rigid_observation`.
Native Fortran uses CMake and the existing manifest verifier. Runtime gates
must be run under the workspace guard before integration; syntax checks alone
do not establish numerical qualification.
