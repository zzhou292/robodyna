# TYPE25 linear finite-offset value contract

The donor is OpenRadioss commit
`a62b27e6baa555d222a580d6218867d0be4d70b5`. Complete original sources, Git blob
IDs, SHA256 identities and exact extraction bounds are retained in
`lib_utest/qualification/type25/native/source-manifest.json`. The C++ code is an
adaptation of selected scalar operations. The Fortran fixtures compile exact
extracted statements and a complete R4EVEC3 body; they are not an OpenRadioss
solver run or an independently converged trajectory.

| Source | Supported operations |
| --- | --- |
| starter RINIT3:344–367,760–775 | Ileng=0 gives XL=1; TYPE25 invokes RMASS and R4BUF3. |
| starter RMASS:64,80–86 | Half of property mass and isotropic inertia at each endpoint. |
| starter R4BUF3:118–123,195–244 | Finite, two-node frame from chord and default/explicit skew seed; nearly parallel Y uses X. |
| engine R4EVEC3:31–end | Current and backtracked midpoint chord, transported transverse axis, mean axial-spin twist, endpoint and midpoint orthonormal frames. |
| engine R6DEF3:234–259,473–478 | Axial length change, incremental radial shear and relative local rotation. |
| engine REDEF3:740,1142–1145 | Linear stiffness and viscous force; native trapezoidal scalar-channel work. |
| engine R6DEF3:353,654–677 | Coupled force criterion, strict failure boundary and clamped output criterion. |
| engine R4CUM3:81–83,109–118 | Force pair and both finite-length shear-arm couples. |
| engine R6DEF3:154–173,438–450; R2LEN3:206–212 | Unscaled elementary mass/inertia timestep, including transverse stiffness times squared current length. |

The selected subset is **Ileng=0, Ifail=1, Ifail2=2**, explicit dynamics with
no small-displacement shortcut, no third node, no coordinate randomization,
sensor, preload/INISPRI, nonlinear curves, rate-dependent failure, hysteretic
stiffness damping or mass scaling. Every source property is resolved outside
this module. Blank keyword parsing, generated IDs, converter defaults and CFG
inheritance belong to the authenticated app source adapter; they are not
inferred from zero-valued TL properties.

All public values are SI. Required working-unit factors retain the interpretation
of the source's bare floors. Geometry floors are converted from working length;
the unscaled native dt formula converts M/J/K/C/length to working units, executes
the native regularizers, then converts time back. Normalized frame-vector floors
are dimensionless. All resolved stiffnesses, M/J and signed threshold magnitudes
must be positive and finite; damping is nonnegative. Unrepresentable results,
degenerate frames and invalid inputs reject without publishing an output.

The four channels are axial displacement, radial transverse displacement,
torsion, and radial transverse rotation. Shear and bending use scalar magnitudes
for their constitutive work, including the native previous force magnitude.
This must not be replaced by vector dot work during a change in direction.
`internal_work_J` is native signed channel work, not a fracture-energy or
dissipation-only ledger. The separate endpoint wrench identity includes the
half-length shear moment at **both** endpoints.

`Evaluate` consumes endpoint positions with the actual interval's midpoint
velocity and spin, and the duration of that same interval. It advances no clock,
authenticates no raw pointer, and grants no nodal or publication authority. The
reference is the qualified startup geometry. Accepted history supplies the
prior transported axis, local displacement/rotation, force/couple cache, channel
works, active flag and failure criterion. The candidate is computed separately.

The strict source failure test is `sum(alpha*(load/limit)^beta)>1`. At exactly
one the spring stays active. Coupled failure is checked after forces and channel
work have been computed: the newly failed evaluation retains that force cache.
The following evaluation applies OFF=0. A collection must preserve this timing,
along with rejection rollback; it must not zero the failing interval early.

The returned elementary dt uses the property's own M/J, not the live combined
nodal mass. Its rotational stiffness is `max(Ktorsion,Kbending)+Kshear*length²`.
A caller must apply its declared safety factor and combine this bound with the
shell, contact and nodal-owner guards. An element dt value is not a stability
proof for a complete connected vehicle.

Qualification covers 64 prescribed input intervals in three working-unit
systems, all frame/deformation/local/native work/wrench/dt fields, strict failure
and post-failure timing, independent long-double angular momentum/virtual work
and dt formula checks, startup M/J/identity/cap admission, and atomic failure.
Source-SI channel force/work checks use binary64 ULP equality. Conversion to
source-mm coordinates follows a different rounded subtraction path; only force
comparisons receive the explicit geometric conditioning enclosure
`64 eps * coordinate_scale * (Kmax+Cmax/dt)` (and its moment arm). This is an
oracle conversion allowance, not a production mechanics/energy tolerance.

The native wrappers bind one element and explicit source parameters. Their
small named COMMON blocks only supply DT1, version and small-displacement flags
to the unchanged frame body; tests must execute serially. Native deformation,
linear response, coupled failure, mass, timestep and scatter snippets preserve
the source statements. The surrounding dimensional packing and radial-channel
selection are equivalent test adapter algebra, explicitly not copied donor
operations. The independent moment/power checks use separate long-double world
vector algebra. CUDA parity tests are a separate required gate before a resident
collection is admitted.
