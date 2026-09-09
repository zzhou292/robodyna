# B2: first force-driven elastic shell coupon

Proposal, 2026-09-09. **This coupled case has not been implemented or run.**
The prescribed Q4 force checkpoint and **B1 mass/optional nodal-rotation
foundation are qualified within their stated scopes**. This document specifies the next small
executable case, with an explicit numerical envelope. It does not claim a
global nonlinear stability theorem, general shell dynamics or Yaris readiness.

## Deliver one observable bending trajectory

Use two coplanar rectangular Q4s, six physical nodes and one clamped short edge.
The strip is **0.2 m long, 0.1 m wide and 0.02 m thick**, with the existing
reference material **E=1.2 MPa, nu=0.3, rho=1,000 kg/m3**, shear factor 5/6 and
drilling stiffness factor 0.01. This is a declared synthetic elastic coupon;
these inputs are not a source-Yaris material or an elastic override of that car.

| Node | Reference position, metres | Constraint |
| --- | --- | --- |
| 0 | (0.1, 0.05, 0) | Free |
| 1 | (0, 0.05, 0) | All translations and rotations fixed |
| 2 | (0, -0.05, 0) | All translations and rotations fixed |
| 3 | (0.1, -0.05, 0) | Free |
| 4 | (0.2, 0.05, 0) | Free |
| 5 | (0.2, -0.05, 0) | Free |

Element connectivity is `(0,1,2,3)` and `(4,0,3,5)`, matching the natural
`(+,+),(-,+),(-,-),(+,-)` node order and the existing shared-node fixture.
All initial reference nodal frames are identity. Capture the flat reference
once through actual Chrono setup, then initialize the TL owner in a small
deformed configuration. Never call setup again to remove initial strain.

Choose the lowest predominantly out-of-plane constrained elastic mode from the
startup modal check below. Normalize its free-tip displacement to **0.01L =
2 mm**, apply its translational components and corresponding world rotational
increments to the reference, and start with zero linear/angular velocity.
Release with no gravity, applied loads, contact, damping or plasticity. This
gives measurable recoverable bending without a loading-function subsystem.
The first result should approach the flat shape and reverse deflection over
approximately half the measured first-mode period. Choose the exact horizon
after measuring that mode and running a bounded 100-step cost probe.

## Keep the implementation within existing owners

| Owner | Concrete responsibility |
| --- | --- |
| TL elements | Reuse [`ReissnerShellData`](../../Total-Lagrangian-FEA/lib_src/elements/ReissnerShellData.h), [`ReissnerShellForce`](../../Total-Lagrangian-FEA/lib_src/elements/ReissnerShellForce.h), [`ReissnerShellAssembly`](../../Total-Lagrangian-FEA/lib_src/elements/ReissnerShellAssembly.h) and [`ReissnerShellMass`](../../Total-Lagrangian-FEA/lib_src/elements/ReissnerShellMass.h). Add only a small bounded batch launch/diagnostic adapter owning immutable reference/connectivity and scratch; numerical equations stay in these helpers. |
| TL state and stepping | Extend the admission contract of [`ExplicitNodalStep`](../../Total-Lagrangian-FEA/lib_src/solvers/ExplicitNodalStep.h) for a **restricted elastic trajectory**. Reuse [`FENodalState`](../../Total-Lagrangian-FEA/lib_src/solvers/FENodalState.h), its stream, additive assembly, private trial state and one commit. |
| Robo-dyna case | A small `ElasticCouponCase` configures the fixture, initializes mass and reference data, coordinates force/advance/candidate checks, and owns this case's energy/admission record. It neither implements another integrator nor embeds keyword parsing or rendering. |
| Robo-dyna/Chrono reference adapter | Reuse [`ReissnerShellSetup`](../chrono/ReissnerShellSetup.h) for actual immutable setup/section extraction. Put the bounded finite-difference/modal audit next to the numerical reference adapter, reusing Chrono/Eigen math; the production case must not include a GoogleTest fixture. |
| Results and scene | Reuse [`NodalMeshOutput`](../chrono/NodalMeshOutput.h), source-aware `AcceptedSurfaceMesh` and shared archive helpers. The [rendering track](RENDERING_ARCHITECTURE.md) consumes accepted frames independently of integration. |

The existing `PrescribedConstantLoads` admission explicitly excludes shell
forces. Do not pass elastic assembly through it, relabel forces as constant, or
put rotational stiffness into the old translational PSD row bound. The new
case policy must name its source/section/mass identity, fixed time step, sampled
spectral limits, allowed envelope and candidate checks. Step requests bind that
policy to the current owner/base epoch/attempt. The coordinator can declare and
enforce this qualified scope; matching tokens do not prove an omitted force or
validator ran. Do not describe the policy as a general stability certificate.

No artificial mass is introduced. Integrate `m_i=rho*t*integral(N_i dA)` and
physical tangential `J_i=rho*t^3*integral(N_i dA)/12` once. Add both element
contributions at shared nodes 0/3. Retain physical mass at clamped nodes in the
model ledger while passing zero inverse mass/inertia to their constrained DOFs.
Select numerical drilling inertia **J_d=J** explicitly; total inertia is the
isotropic `J*I` admitted by the current angular update. Preserve physical and
artificial rotational energies separately, using current physical directors.

Changing only J_d produces anisotropic total inertia, which the current owner
and gyroscopic-free step do not support. Such inertia-sensitivity runs require
qualified anisotropic dynamics first; they are not a switch in this coupon.
For now report drilling energy/spin throughout the trajectory and retain that
limitation. Do not vary physical tangential inertia to simulate a drilling-only
sensitivity study, or copy Chrono's heuristic quarter-tile box inertia.

## Choose the fixed step from measured mechanics

The coupled system has **24 free velocity DOFs: four free physical nodes with
three translations and three rotations each**. Build its neutral stiffness
by centered differences of the actual coherent Chrono force operation, with
world force/couple rows and free translation/world-spin columns. Convert Chrono
local couples once. Use translation perturbations `1e-6L` and spin perturbations
`1e-6 rad`, then halve both. Cross-check directional derivatives using the TL
force operation. The coherent reference deliberately rejects tangent requests;
do not call its unqualified `ComputeKRMmatricesGlobal` as the stiffness oracle.

With the independently assembled diagonal mass/inertia M, inspect
`A0=M^(-1/2) K0 M^(-1/2)`. At the stress-free reference, check symmetry, positive
clamped modes, mode shape, maximum frequency and perturbation refinement before
using the modes. No unexpected zero mechanism may be dismissed as an hourglass
mode. Proposed pre-run gates are normalized symmetry/directional disagreement
at most `1e-5`, and maximum-frequency change at most **0.5%** after perturbation
halving. Diagnose small negative eigenvalues relative to the finite-difference
error; do not silently project a materially indefinite matrix to PSD.

Sample the chosen deformation mode at amplitudes `0`, `0.5`, `1` and `2` times
the intended initial amplitude. Away from rest, world-couple derivatives can
include rotation-coordinate connection terms. Use a consistent local
exponential-coordinate energy Hessian or the operator norm of the full
mass-scaled force Jacobian; do not erase its skew part and call the result a
proved tangent. Record the largest sampled rate and recheck at saved diagnostic
frames. The initial step proposal is the minimum of:

- `0.1 / sqrt(maximum sampled mass-scaled stiffness norm)`;
- `0.1 * shortest reference edge / c_membrane`, with
  `c_membrane=sqrt(E/(rho*(1-nu^2)))`;
- `0.1 * t / sqrt(12*G/rho)`, with `G=E/(2*(1+nu))`, as an independent
  director/shear inertia scale.

Use an additional factor of two on the sampled stiffness norm when defining
the allowed monitored spectral envelope. These factors are conservative
initial choices for a restricted numerical experiment, not a nonlinear
stability proof. The thin-shell rotary scale can be much stricter than the
edge/wave scale. Record the actual limiting value and use `h`, `h/2`, `h/4`
as **separate fixed-step runs**; never enlarge mass or clamp a failed bound up
to the requested step. If a sampled tangent escapes its envelope, stop at the
last accepted frame and revise/restart the declared experiment.

## Validate each prepared candidate before commit

Reuse the same force operation to evaluate candidate elastic energy and its
chart; add a small geometric/admission check over the two elements. Candidate
checks run on the owner's stream and finish before the case permits commit.
Keep these proposed limits fixed before the first run:

| Quantity | Initial bounded envelope |
| --- | --- |
| Physical relative directors | Pairwise rotation below 0.25 rad, stricter than the force operation's open 90-degree chart; per-step world rotation increment at most 0.01 rad. |
| Motion | Maximum nodal translation from reference at most 0.02L; absolute physical director departure at most 0.1 rad. |
| Element geometry | Positive oriented Gauss-point area relative to the original normal, at least 0.8 of its reference value; area-norm ratio within [0.8,1.2]. Reject degenerate display triangles as well. |
| Strain | Maximum membrane/shear component at most 0.01; maximum `t*curvature` component at most 0.05. All states, strains, resultants, force/couple and energy fields finite. |
| Total energy | Maximum `abs(T+U-E0)/E0` at most 1% over the accepted no-load trajectory; E0 includes the positive initial strain energy. Report all physical/artificial terms. |
| Time refinement | Common-time tip displacement, first extremum timing and energy envelope change at most 5% between h/2 and h/4, with absolute floors for zero crossings. Investigate rather than divide by a near-zero displacement. |

The recorded spectral and per-candidate checks together define the admitted
trajectory; none alone certifies arbitrary nonlinear motion. Keep a diagnostic
of the conservative force/potential increment
`delta(U) + F_n dot delta(x) + couple_n dot delta(theta_world)`. Its expected
nonzero second-order term must be assessed using the measured stiffness and
increment size, with an explicit roundoff floor; do not require exact zero.
Use the proposed bound `0.75*lambda_envelope*delta(q)^T*M*delta(q) +
128*epsilon*E*t*area`, where delta(q) contains translation and world rotation
increments, lambda_envelope is the doubled sampled stiffness norm, and epsilon
is binary64 machine epsilon. Record every term and freeze this dimensional
budget before running; failure stops the case rather than widening the budget.

For the isotropic velocity-first update, kinetic-energy change has the
independent discrete identity
`delta(T) = h*sum(F_n dot (v_n+v_new)/2 + couple_n dot (omega_n+omega_new)/2)`
over free DOFs. This is distinct from force times the velocity-first coordinate
increment. Check it separately from total-energy drift. The proposed kinetic-work
residual budget is `128*epsilon*max(E0,T_n,T_new)`; report the unscaled residual
as well. Fixed constraints do zero work; their reported reactions belong to the
accepted **base** state.
Check linear impulse and total numerical angular impulse using those reactions
and the base-position moment arms. The rotational momentum uses the declared
total isotropic inertia; it must not be labelled purely physical shell spin.

A rejected late candidate preserves accepted positions, quaternions, velocities,
reactions, energy ledger, time and visible frame. Test a rejection after a
previous successful step, then compare the complete retry result with a clean
second run. A future material/contact history participant would require its
own coordinated transaction; this elastic case has no such hidden history.

## Execution and visual exit

Start with the existing guarded startup/one-step gates, then a 100-step cost
probe, then the measured half-period horizon and its two refinements. Keep
two CPU affinity slots, one numerical/build worker and the shared GPU lock;
set per-run memory/time limits from the tiny batch forecast. Compute forces
and candidate checks on CUDA. Read back complete states only for declared
modal/diagnostic/output frames, with small aggregate status/energy diagnostics
otherwise. Avoid optimizing away provenance checks or retaining whole runs in
device memory to improve this first timing result.

Exit evidence is a reproducible accepted result bundle with displacements,
directors, force/strain/energy histories, timestep admission and failure tests.
Then pass rendering R1: a fixed camera must visibly show the **actual** bending,
changing silhouette/normals, clamped edge and recoverable motion, with simulation
time displayed. Use the same accepted geometry in replay and live output; a
prescribed animation or mesh archive alone is not that visual gate. Missing VSG
dependencies may leave R1 pending while coupled numerical work proceeds.

## Qualified prerequisites

B1 passes [14 CUDA nodal-rotation tests](../../crash-work/reports/rotary-foundation-tests-1/utest_nodal_rotation_cuda.xml),
[ten host mass tests](../../crash-work/reports/rotary-shell-tests-1/utest_reissner_shell_mass.xml)
and the extended [Chrono accepted-output checks](../../crash-work/reports/rotary-output-tests-1/crash_tl_nodal_output_check.xml).
That evidence covers isotropic prescribed-load stepping, component constraints,
world rotation/torque covariance, discrete kinetic work, complete failed-attempt
retry, separated physical/artificial inertia and accepted rendering geometry.
It does not admit state-dependent elastic forces through the constant-load
policy, qualify the B2 trajectory or establish a graphical VSG rendering result.
