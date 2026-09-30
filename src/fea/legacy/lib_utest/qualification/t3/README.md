# T3 host/CUDA startup and prescribed-rate port

Source staged for review; no port build or numerical execution is
claimed. The retained native R1/R2/R3 remains the independent callable reference.
This gate ports startup and prescribed geometry/rates only. Force/history is
the next stage after these checks pass; no dynamics, contact, timestep or
original MAT024/NIP3 behavior is admitted here.

## Ownership and selected equations

`elements/t3` owns fixed three-node `ReferenceData` and `Kinematics` values.
`InitializeReference` stages the native starter frame, selected C3DERII
characteristic length/zero ISMSTR-1 derivative slots, selected C3INMAS
angle/pi masses and A/4.5 inertia, and SPMD_MSIN contributions. Actual total
inertia preserves its native expression; physical/added partitions are
diagnostics and never reconstruct that total.

`EvaluatePrescribed` composes IRESP2 endpoint gathering, current C3EVEC3
IFRAM_OLD1, C3DERI3 and complete selected C3DEFO3/C3CURV3. `T3Geometry.h`
shares only the starter/current frame arithmetic that is exactly identical.
`T3CurrentGeometry.h` owns engine derivatives; `T3Rates.h` owns the native
quarter-step corrections and geometry-dependent angular shear, without a Q4
surrogate. All38 R2 observables and explicit endpoint/midpoint timing remain.
`GeometryWork` is private element scratch for the later force stage, not a
second model/history/time owner. Future material porting can reuse the pure
`materials/ShellElasticLaw1.h` point operation after T3 packing/work is mapped;
no material helper is needed in this startup/rate stage.

No native Fortran, allocation, CUDA API or state owner appears in production
headers. Shared Vec3/Matrix3 and finite/quaternion helpers remain owning math
utilities. The qualification executable alone links the frozen native oracle.

## Numerical admission frozen before execution

The GPU uses binary64 preflight. It cannot reproduce the native host's
long-double comparison at cutoff boundaries. Set `b=64*DBL_EPSILON`; all
coordinates are finite with each absolute component<=1e6m. Form the same
rounded world edge differences as the native frame, their ordinary binary64
norms `L_i`, and `L=max(L_i)`. Admit only:

- `1e-9*(1+b) < L_i < 1e3*(1-b)` for every edge;
- `min(L_i)/L > 1e-6+b`;
- `norm(cross(edge01/L,edge12/L)) > 1e-6+b`.

The additive bands apply to dimensionless unit-scale ratios; the last
criterion is not a relative epsilon band on the small area cutoff. The two
length bands have physical units and scale with their respective thresholds.
Native frame arithmetic receives the original unscaled coordinates; normalized
edges are used only for preflight. This is a conservative numerical domain,
not an interval proof of every geometry, exact shell-quality certificate or
wall-projection rule. Exact edge-on orientation relative to any wall is valid.

Startup checks each actual native law-of-cosines argument before ACOS:
`abs(c)<=1-1e-12-b`. No clamp/angle renormalization is performed. Engine
C3DERI3's actual ordered dot product must give
`Y3 > 32*EM15+b*L`, where native `EM15=1/EP15`. Its original signed floor
expression is retained but remains inactive. Both C3DEFO3 denominators must
be nonzero and all published results finite. Positive masses/inertias must
remain representable; underflow is rejected. Output frame orthonormality and
positive determinant use the original native wrapper's5e-13 tolerance.

Clearly inside, outside and ambiguous cutoff fixtures test rejection/output
preservation. They do **not** claim identical cutoff acceptance with the native
long-double guard. All safely admitted arithmetic and native comparison
budgets remain fixed. A finite positive h must retain h/4>0 and strict
`base < base+h/2 < base+h`; sample index is nonzero. Every failure preserves
caller output and immutable reference; public value records assume ordinary
producer ownership rather than authenticating arbitrarily forged memory.

## Verification and fixed budgets

Tests compare the same safely admitted operands to native R1/R2, preserving
all startup fields and all38 rate observables. Port/native comparison uses
`2e-12*(dimensional_scale+abs(native))`, as already qualified for the QEPH
host/device ports. Independent long-double startup/affine/angular oracles
retain R1/R2's2e-12 budgets: frame/angles1, geometry L/L², positive mass/inertia
their own scale, raw first five `L*V+L²*omega`, raw curvature `L*omega`, and
normalized scales divided by area. World covariance retains2e-11. No tolerance
is inferred from a first failed run or changed because of a device result.

Ten host functions cover analytic scalene/right/equilateral masses and all18 rate
columns, coordinate/section scaling, proper rotation/cyclic/reversed ordering,
shared native mass accounting, quarter-step rigid-spin characterization,
staged malformed/cutoff rejection and late overflow/underflow with clean retry.
Three actual CUDA functions run these same owning header operations on device records,
compare their complete results to native references and independent truths,
and check failure output bytes plus clean retry. Missing CUDA fails, not skips.
No device limits are raised. Root serializes builds/runs with the existing
workstation guard and retains measured kernel resources separately.
