# Native rigid-group step packets

`NodalRigidFrameStep.h` and `NodalRigidGroupStepMath.h` implement allocation-free
host/device values for the pinned OpenRadioss free, 3D, explicit rigid-body
branch with more than two members. They do not attach constraints to an owner,
advance a physical clock, publish accepted state, integrate nodal quaternions
or authenticate a source model. Every function leaves its output unchanged
when validation or arithmetic fails.

The caller first supplies the complete force/couple wrench through the existing
`AggregateWrench`. `EvaluatePrimaryStep` saves angular velocity expressed in
the previous principal frame, applies native `ROTBMR` with `previous_drift_dt`,
then uses that saved velocity and the newly transformed torque for Euler's
anisotropic equations. The primary proxy-J multiply/divide is algebraically
cancelled; comparison with the native pseudo-moment path permits the declared
binary64 tolerance. Native member total J is not added to body principal
inertia by these packets; aggregation belongs to the startup model.

`EvaluateMemberStep` consumes the matching primary input/result and a member's
native mass, total scalar J, state and assembled loads. It computes native
member accelerations, `m*a-F` / `J*alpha-C` reactions, common velocity kick and
second-order positional drift. The following relative update is its geometric
interpretation, not a replacement for the retained acceleration/kick order:

```
r_plus = r + drift_dt*(omega_plus cross r)
           + drift_dt^2/2*(omega_plus cross (omega_plus cross r))
```

This update has finite-step distance drift. No projection or quaternion
renormalization of member positions is performed. `ROTBMR` retains its own
angle-squared floor at `1e-10`, separate normalization of two axes and cross
product construction of the third. Zero duration still executes that arithmetic.
The greater-than-two-member old-spin guard rejects
`drift_dt^2*|omega_old|^2 > 1`; a future owner adapter must also apply its case's
new-spin limit and complete numerical admission.

The three durations remain distinct caller-supplied values. Previous drift is
nonnegative; kick and next drift are positive. The packet performs no scheduling
or check that a donor engine chose those values. Tests include ordinary equal
steps, unequal durations, and `(0,h/2,h)` explicitly labeled as the proposed TL
physical-rest startup mapping. Passing that packet does not establish native
engine startup or the principal frame's actual temporal phase in a running
owner. Those are subsequent recurrence-integration gates.

Qualification targets in `lib_utest/qualification/nodal_rigid_group` are
`nodal_rigid_step_check`, `nodal_rigid_step_native_check`, and optionally
`nodal_rigid_step_cuda_check`. The native directory retains six complete pinned
files and fourteen exact fragments. Authored wrappers compose native frame,
gyro, member acceleration, reactions, common kick and drift without native
global state. The existing source verifier checks every original hash/blob and
extraction range. Host/native/CUDA comparisons use the existing `2e-12` scaled
binary64 tolerance; whole-packet late rejection and same-backend retries use
unchanged bytes or exact values. Independent analytical tests cover spherical
kick/drift, Rodrigues comparison, second-order distance growth and impulse
reactions. All compilation and numerical execution remain subject to the
workspace resource guard.
