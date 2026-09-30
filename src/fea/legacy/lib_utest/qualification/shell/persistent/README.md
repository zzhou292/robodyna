# Persistent Q4 qualification tests

This fixture exercises the combined, pinned K1/K2/K3 path through the real TL
physical-node views. It is a prescribed-kinematics numerical fixture, with at
most two quadrilaterals, eight physical nodes and 64 accepted increments in the
longest test. Root owns implementation, builds and execution; this source-only
test preparation is not a recorded passing gate.

The tests use independent plane-stress elasticity, exact three-point bending
integration and boundary-traction/couple integration. Coefficient tables and
kernel B matrices are not copied into the force oracle. The fixed-geometry
load/unload/reload and split-increment cases are constitutive-operator tests:
their prescribed velocities do not describe advancing nodal positions. Their
trace-free membrane/bending fields keep physical thickness unchanged, allowing
an exact elastic stored-energy oracle. Explicit transverse-shear checks retain
the declared section shear factor. Normalized FOR and MOM are restored using
the same step-thickness snapshot used by K2.

The ITHK comparison separately checks that physical thickness changes even when
the normalization snapshot is fixed. With elastic increment de_xx and
alpha=nu*de_xx/(1-nu), ITHK0 gives t_n=t_0*(1-n*alpha); ITHK1 gives
t_n=t_0*(1-alpha)^n and uses t_(n-1) during that step's K2/K3 assembly. This is a
source-compatibility check of an incremental update, not a claim that finite
load/unload exactly reverses a multiplicative thickness approximation.
The changing-geometry test prescribes a uniform stretch with exact endpoint and
secant velocity data. Current geometry uses area A_0*lambda and increment
d(lambda)/lambda at the interval end; frozen geometry retains A_0 and d(lambda).
These distinct source conventions are tested explicitly, without claiming
finite-strain accuracy from their difference.

Each rejected geometry/material/assembly trial must preserve every published
nodal and element field, time and epoch. Retry is compared with a clean trial.
Finite transverse rates of magnitude 1e200 additionally trigger an actual
post-kernel invalid output and must invalidate the earlier trial while preserving
accepted bytes and a clean retry. Trial tokens must not outlive their owner;
cross-owner, stale-epoch and superseded-attempt checks exercise live owners only.
Shared-node and element-order tests check one physical node space and additive
force/couple assembly; they do not yet qualify full production batch scheduling,
mass, time integration, contact, plasticity or fatal-device recovery.

The exact rigid endpoint/secant-velocity trajectory is deliberately labeled a
**characterization with an unresolved physical acceptance issue**. For a rotation
delta=Omega*dt about local y, inspected K2 algebra yields
de_xx=cos(delta)*(cos(delta)-1) and dgamma_xz=delta-sin(delta), rather than zero.
At fixed duration the accumulated membrane residual is first order in dt and
shear residual second order. The test records strain drift, spurious work and
force, checks refinement, and applies conservative small-angle experiment caps
(3e-6 membrane strain, 1e-8 shear strain and .002 J work). These caps contain this
probe; they are **not production objectivity acceptance thresholds**. A pass
does not qualify stress-free finite rigid motion or authorize a crash coupon.
Its machine-readable test property is `production_objectivity_qualified=false`.
That production requirement remains open until the integration/formulation
choice resolves or explicitly justifies the measured finite-step error.

Deterministic arithmetic checks use declared absolute floors plus 2e-9 relative
tolerance: typically 2e-12 strain, 2e-5 Pa point stress, 2e-7 N force, 2e-9 Nm
couple and 2e-9 J work; rotated/shared reductions allow the locally stated larger
absolute roundoff floors. Rigid-trajectory source-algebra comparisons use their
separately stated cancellation/refinement bounds. No tolerance is selected from
an observed failing run. Logs/XML and resource evidence must be supplied by the
bounded build/runtime job before any promotion.
