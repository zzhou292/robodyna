# TYPE13 H1 force values

`InitializeForce` and `Evaluate` compose the qualified TYPE13 startup values with
six native H1 force histories. They allocate nothing and publish only their
completed output value. They create no owner, clock, accepted slab, source
connection, tie, nodal mass or integrator. Original source cards remain the app's
responsibility.

`NativeEndpointKinematics`, `NativeFrame`, `NativeChannelHistory` and
`NativeHistory` are explicitly **working-unit** values. This preserves the
source coordinate bits and the REDEF3 division by original length followed by
restoration of dimensional history. Translation deformation and accumulated
plastic deformation are native lengths; rotational counterparts are radians.
Per-channel forces are native force or moment; all six work histories have
native force-times-length units. Each channel owns its search position,
unmasked FXEP and masked FX independently, including the two pairs sharing
immutable source curves. Only the separately named SI outputs are converted.
The current ABI uses 328 bytes/history and 720 bytes/evaluation on the qualified
64-bit host; this is a value-storage measurement, not resident capacity admission.

The explicit fresh packet requires original endpoint positions, uniform
translation and zero spin, with TT=0/DT1=0/INISPRI=0. R4EVEC3 still normalizes
its frame at zero duration. The returned history is the initial force cache.
Subsequent packets use TT>0, positive `native_dt` (native DT1), current endpoint
positions and the actual interval midpoint velocities/spins. The caller supplies
and authenticates this phase; arbitrary supplied values cannot establish an
engine trajectory. There is no inferred timestamp or duration.

The admitted property is the startup module's exact Ileng1/H1 scope: four owned
five-point curves, six references, unit A/LSCALE, zero damping/secondary curves,
Ifail1/Ifail2=0 and no sensor/rate-dependent failure. The explicit dynamic
REDEF3 branch still evaluates its rate, max/log factor and zero damping terms.
The shorter implicit expression is not substituted. VINTER2 retains native
search direction, strict knot comparisons and end-segment extrapolation.

Each channel evaluates force and signed trapezoidal internal work against the
previous OFF. The six squared displacement/curvature failure contributions then
accumulate in source order; native Ifail2=0 sets alpha=1/beta=2 independently of
the stored property declarations. Criterion **>=1** disables the next interval.
The failure interval retains its current force. A subsequent evaluation masks
current force to zero, but its work still includes the previous cached force.
Native unmasked FXEP and plastic/search histories continue their H1 update after
OFF=0. This signed work is neither plastic dissipation nor a full energy balance.
`newly_failed` is an event value, not a second clock or owner publication.

R4EVEC3 transport and R4CUM3 force/couple projection are shared with TYPE25;
TYPE25's public wrappers preserve their existing admission and scalar operation
order. The two opposite endpoint forces include both finite-length shear moment
arms. TYPE13 stiffness preparation remains separate: source lineic M/J are
multiplied by original length; the six stiffnesses and current shear arms are
normalized by original length before native R2LEN3. The returned critical dt is
unscaled and still needs a future contributor's safety/admission policy. Added
J is already in the qualified property total and is not added a second time.

All public checks and arithmetic occur before output assignment. Bad history,
unsupported startup, collapsed geometry, nonfinite intermediates or late channel
failure leave the whole output unchanged. Accepted history may alias the output's
history for a successful value update; rejecting a packet never changes it.
No byte-serialization promise is made for padding; qualification compares every
active field explicitly.

Native authority is pinned revision `a62b27e6baa555d222a580d6218867d0be4d70b5`.
`qualification/type13_recurrence/native` retains complete R4DEF3, REDEF3, R2LEN3
and VINTER source files plus exact hashed excerpts. The independent oracle reuses
the existing unchanged TYPE25 complete R4EVEC3 and R4CUM3 source wrappers. Main
anchors: R4DEF3:190–218,262–289,566–573,337–378,800–827,870–877;
REDEF3:272–286,432–443,763–776,1018,1141–1146,1153–1159;
VINTER2:133–192; R2LEN3:205–213. Precision authority remains the startup donor's
`constant_mod.F`: EP30=EP20*EP10 at638, EM30=ONE/EP30 at663, with double-precision
`my_real`. EM15/EM30/EP30 are working-unit constants, not SI regularizers.

H0/H2…H12, damping/rate/sensor/force failure, preload/INISPRI, small-displacement,
noise/collapsed geometry and nonzero endpoint releases are outside this slice.
So are batch/owner admission and the original TYPE2 Spotflag28 shell attachments.
A free beam force oracle does not restore those source load paths.
