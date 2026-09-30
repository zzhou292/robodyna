# CW1 broadside model and contact probes

Prospective source-only increment, following the retained native-map/PowerGram
support. The old helpers, their README, source maps, and BQ4 policies are
unchanged. No spectral, switching, startup or impact admission is implemented.

The immutable model is exactly the tuple frozen in
`planning/QEPH_WALL_RECURRENCE_SCREEN.md`: one/two 20 mm square cells in world
Y/Z, native reference X=-0.000375/4 m, touching probe baseline X=0; E=200 GPa,
nu=.3, rho=7890 kg/m3, t=.001648 m. Native m/J and all history stay separate
from the owning Q4 reference-area weights. The reused `Square()` wall is the
two-triangle synthetic +/-2 m fixture, not canonical Yaris geometry. The whole
fixed +/- .05 m Y/Z motion box is checked with clearance 1e-6 before probes.

Kappa is the upper endpoint of the recorded six-stage interval chain
rho, rho*t, rho*t*8, rho*t*8*8, previous/.000375, previous/.000375, using owning
Scale/DividePositive operations. The maximum depth remains .0005 m; per-parent
force/energy errors remain 5e-7 N and 1.2500000000000005e-12 J. Every used area,
stiffness and k/m certificate is retained. The independently assembled native
mass and actual nominal stiffness define k/m; its outward relative spread must
be <=1e-12. There is no fitted coefficient or added inertia adjustment.

`BuildContactKick` retains all 109/194 normalized coordinates and adds only
the negative world-X position-to-velocity coefficient. `BuildContactBranch`
multiplies the full native shell matrix on the left: A_shell*B_contact.
The actual probe evaluates the owning host contact law before adding its kick
to copied normalized velocities and calling the retained native map. No new
force cache, physical owner, initial-force call or accepted clock is present.

Only h in the six frozen values and physical boosts {-8,0,+8} m/s are accepted.
Actual masks must be wholly touching/active or wholly inactive. The n+7
directions have strict positive/negative normal displacements; the baseline at
zero gap is retained separately. Three amplitudes 2^-18, 2^-19, 2^-20 produce two
one-sided order-two quotients. All raw samples precede numerical decisions.
The matrix budget stays 5e-8*max(1,||A||infinity*||d||infinity). The reported
residual is rounded upward, its comparison budget downward. Compact records
reuse the owning point/parent types and retain every used certificate.

Focused tests use the existing 2e-12*max(1,|expected|) normalized analytic budget,
independent long-double represented-coordinate area/mass/force/potential
calculations, exact immutable field/default checks, and the frozen directional
matrix budget. Expected rejection must preserve complete caller output. Test
execution and any remaining numerical uncertainty belong to the parent guard;
no threshold can be widened after observing a failure.

The matrix argument is an explicit numerical operand, not an authenticated
source record. The future screen must bind it to the same immutable model, h,
amplitude and physical boost and retain the complete native probe. Passing a
finite collection of directions alone cannot authenticate an arbitrary matrix.

New numerical sources: WallRecurrenceModel.cpp, ContactBranchOperator.cpp,
ContactNativeMap.cpp, ContactBranchProbe.cpp. They link retained recurrence
support/native Q2 and owning tl_nodal_wall_contact, without CUDA or a solver
runtime. New tests are separate from the ten retained support functions.
