# QBAT four-point reference and geometry values

This allocation-free leaf selects OpenRadioss IHBE11, isotropic TYPE1/LAW44,
IREP0, ISMSTR2, NPTR2/NPTS2/NPTT1, one layer, IDRIL0, NPINCH0, centered section,
ITHICK1/IPLAS1 and unscaled native mass. It addresses the original 4,250 ELFORM9
midlayer **quads**. Four in-plane material points coexist with one thickness
point. The part's remaining triangle belongs to a separate native T3 path.

`InitializeReference(input, reference)` accepts SI positions, density, Young's
modulus, Poisson ratio, thickness, resolved virgin PM(24) and resolved CNDLENI
viscosity fields. The material producer owns PM(24); this leaf does not read
keywords or choose an analytic law, rate policy, failure threshold or history.
The initial source material has rho=1000 kg/m³, E=250 MPa, nu=.35 and t=.5 mm.

`Reference` is a copyable immutable packet. Its quadrilateral values reuse the
existing CNEVECI/starter CLSKEW3, CDERII and centered CINMAS FAC12 leaf unchanged.
This shares native arithmetic, not a QEPH force-law declaration. The native
total inertia is evaluated once in its source order, and physical/added
inertia remain diagnostic partitions. CNDLENI has its own IHBE11 **4/3**
characteristic-length factor, characteristic length, sound speed, unscaled
element timestep and equal four-node translational/rotational stiffness
contributions. It does not compute a combined nodal timestep.

`EvaluateGeometry(reference, current, geometry)` requires the active OFF=1
current-position path. It uses the existing pure engine CLSKEW3 leaf, distinct
from starter normalization. It retains current world axes, signed actual ZL1
warpage, centered projected positions, native VCORE, all four JAC/HX/HY rows,
CBADEF1 BM(1:8), and the constant CBADEFSH shear coefficients. The point order
is the native outer IS, inner IR order. NPTT1 forces the native flat algorithm
even for warped coordinates; the actual warpage remains visible. The stored
BM is not a replacement standard bilinear shear operator.

The existing conservative projected-convexity and finite frame domain applies.
All four signed pre-ABS Jacobians must be positive. Invalid options, missing
reference, invalid/nonfinite coordinates or nonfinite coefficients preserve
the complete previous output. Successful publication consumes all inputs
first; reinitializing from `reference.input()` is supported. Public value
types do not authenticate a source, owner, phase, allocation or transaction.

This increment has no velocities, DT1 frame correction, deformation rates,
material points/history, force/work, failure removal, batch, third contributor
or source admission. Later integration must retain the exact current and
saved geometry branches and distinct force-stage time inputs. The native
force caller's damping/default behavior needs its own qualification.

Source authority and explicit defaults are recorded in
`lib_utest/qualification/qbat/SOURCE_CONTRACT.md`. The owning qualification has
host, pinned Fortran, original-source and optional actual CUDA checks.
