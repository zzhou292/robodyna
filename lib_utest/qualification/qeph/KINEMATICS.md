# QEPH Q3b prescribed geometry and rates

The corrected Q3b gate passed all seven host and three actual-CUDA functions
at the original budgets. The first execution passed five of six host and two
of three CUDA functions; native-parity comparisons failed on characteristic
length because four native default-REAL literals had been ported as binary64
literals. The correction preserves their actual native kind. Q3a startup passed seven
functions and is retained independently at TL
`bf57436`, checkpoint `qeph-q3a-startup-1`. Q3b does not change its startup
equations, tests, or tolerance. The shared status enum only appends
`kInvalidReference`, preserving prior numeric values.

The public API is `QephKinematics.h`:
`EvaluatePrescribed(const ReferenceData&, const PrescribedInterval&, Kinematics&)`.
Inputs are endpoint world positions and midpoint world velocities/angular
velocities with explicit base time, positive dt, and caller sample label.
Output uses named fields matching all 80 native Q1 observables and their
planar flag/time labels. It is not a serialized ABI, physical owner, force
cache, element history, time integrator, or zero-step initial-force operation.

## Selected coherent donor path

All adapted source retains OpenRadioss revision
`a62b27e6baa555d222a580d6218867d0be4d70b5` and AGPL/Siemens provenance.
The preserved v2 `kinematics-source-manifest.json` lists exact original file
identities and transformations. The live v3 manifest adds Q3c force/history
source without changing these qualified geometry/rate expressions or tests.

| Owning header | Selected native work |
| --- | --- |
| `QephCurrentFrame.h` | CZCORC1 gather/geometry and engine CLSKEW3 IREP0 |
| `QephVelocityCorrection.h` | World rates, midpoint correction, final normalization |
| `QephProjection.h` | CZCORP5 planar switch and complete explicit warped projection |
| `QephProjectionInverse.h` | CZCORP5 inline IRESP2 six-entry inverse and floors |
| `QephRates.h` | CZDEF eight regular and six hourglass rates, warped curvature additions |
| `QephGeometryWork.h` | Private coherent intermediates retained for the later force chain |
| `QephKinematics.h` | Bounded validation, composition, and staged publication |

The fixed context is IREP0, IRESP2, ISMSTR-1, IMPL_S0, IVECTOR0, NPT0,
IDRIL0, IXFEM0, IDRAPE0, IDT1SH0 and active OFFG1. CORTDIR3 is an actual
no-op on this branch, not a substitute callback. The IRESP1 A3INVDP_V call,
drilling-only projection, small-strain, XFEM and inactive branches are not
admitted. The engine normalization differs from the starter; each keeps its
original arithmetic and floors. No classic-shell projection is substituted.

Current geometry, corrected rate differences and projection feed one private
work record; the later force chain can consume those intermediates directly.
The public raw warpage is preserved before the donor switch
`Z1² < LM*1e-8`. Effective warpage is zero on the planar branch; local normals
report +Z and unused DI/DB report zero, exactly as the native public adapter.

## Input and failure contract

ReferenceData is the caller-immutable successful startup result. Sanity checks
reject missing preparation, invalid material/IDs, improper frame, nonfinite
stored geometry and nonpositive mass/inertia. They do not authenticate
coordinated finite mutations or recompute complete startup on every call.
Current positions must satisfy the Q3a determinant exclusion and conditioned
projected convexity domain using actual engine arithmetic. All observable and
retained work values must be finite; area, reciprocal area, characteristic
length and nodal factors must be positive. Every failure leaves the complete
caller output bytes unchanged; reference and interval are never modified.

Base time is finite and nonnegative; dt is finite positive and the represented
endpoint must be finite and strictly greater than the base. The sample index
is an arbitrary uint64 label, never implicitly incremented. This stronger
advancing-time admission is explicit and does not imply native restart or
physical epoch ownership.

## Budgets frozen before execution

Native/host/device parity uses
`abs(actual-reference) <= 2e-12*(dimension+abs(reference))`.
Independent covariance uses the existing Q1 coefficient `2e-11` separately.
L is maximum current node0-relative distance; R is
`max(1/s, max|omega_i|, max|v_i-v_0|/L)`.

| Fields | Dimension floor |
| --- | --- |
| Frame, local unit normals, FACN | 1 |
| Area / reciprocal area | L² / 1/L² |
| Position, raw/effective warpage, characteristic length | L |
| Projected omega, first five regular rates | R |
| Last three curvature rates | R/L |
| Hourglass components 0,1,4,5 / components 2,3 | R*L / R |
| DI/DB | Numerical unit floor in fixed SI coordinates |

DI/DB are the donor's mixed translation/rotation projection coefficients;
their source matrix adds 4 beside squared coordinates. They are not presented
as a uniform physical inverse-length tensor. The declared numerical floor is
explicit instead of inventing a dimensional certificate.

Seven host and three actual-CUDA functions passed. Each native-parity function
checks 216 configurations: nine startup geometries, four cyclic orderings,
two common world transformations, and static/mixed-affine/general-rigid rates.
Independent checks include all eight affine rate components at metre and
10/20 mm scale, exact finite-step corrections, bilinear saddle normals,
two observable planar-switch cases, general-axis/Z-spin rigid truth,
covariance and malformed/late-arithmetic failure with complete byte preservation.
Pure prescribed twist `omega_y=beta*y` has native transverse hourglass
component 4 equal to `4*A*beta`; affine rotations do not imply zero hourglass.

The rigid fixture uses h=.04/.02/.01 s and preserves Q1's exact gradient
oracle, including the real O(h) general-axis transverse shear residual.
Neither a false finite-step zero nor a higher-order dynamics claim is imposed.
GPU tests use one reusable packet below 4 KiB, one thread, canaries, and actual
native plus host comparisons. CUDA absence is failure. No numerical budget,
vehicle coverage, production force admission or throughput is inferred here.

## Retained first-run diagnosis

`crash-work/checkpoints/qeph-q3b-first-execution-1/manifest.json` preserves the
failed sources, binaries and reports; SHA-256 is
`4412beeaf91f95fc9aebec97cfac8a7a2b32d56b3fb37634000173ed3f915bb8`.
The host native-parity test reported exactly 216 mismatches, one per fixture,
in characteristic length. Source CZCORC1 uses default-REAL constants 3.413,
0.7071, 0.78 and 0.22. MYREAL8 controls variable declarations, not these literal
kinds. Their promoted values must match the actual native compiler contract.

For the metre rectangle, the binary64-decimal implementation gave
1.0120397997335147 m; promoting the four binary32 literals gives
1.0120397749272299 m. Their difference is 2.4806284804057555e-8 m, matching the
first XML mismatch against the unchanged 6.496215504854039e-12 m allowance.
The port now explicitly promotes the four `float` literals to double. A new
independent rectangle oracle uses their exact hexadecimal values in long
double; host and device checks retain the original field budget. Every parity
comparison now reports its field name/index and full-precision values.

The remaining selected raw numeric literals are exactly representable; named
EM10/EM20 and FIVE_OVER_4 come from the native MYREAL8 constant module. No
native file, geometry convention, rate formula or test tolerance changed.

## Corrected execution evidence

Reports under `crash-work/reports` are `qeph-q3b-literal-build-1.json`,
`qeph-q3b-literal-host-tests-1.json` and
`qeph-q3b-literal-cuda-tests-1.json`, all with status passed. The host XML at
`qeph-q3b-literal-host-xml-1/qeph_kinematics_port_check.xml` records seven
tests, zero failures/errors, and 216 native-parity configurations; SHA-256
`08af090ba6cba9962884aacc08d9b6963daf7979f514606f54153d0f5b8c779f`.
The CUDA XML at
`qeph-q3b-literal-cuda-xml-1/qeph_kinematics_port_cuda_check.xml` records
three tests, zero failures/errors, and 216 configurations; SHA-256
`4ce09560b5aaa67a59de6e75b8e69556c6c10650cc9f7f822d0ce25bb0c4ecd6`.

The build took 4.514 s with 356,057,088 bytes peak sampled RSS. Host and CUDA
guards took .255 s and .453 s respectively; the CUDA GTest duration was .245 s.
CUDA records exactly 1,528 owned device bytes and one kernel thread. These
process timings are not kernel-throughput measurements, and sparse process
samples do not establish peak CUDA-context allocation. The guard used two CPU
affinity slots, one test/build job, at least 32 GiB available RAM, at least
8 GiB free GPU memory, and a 4 GiB device-wide growth cap for CUDA.
The independent characteristic-length regression passed at the same field
budget, including its host/device rectangle checks. Force/history, temporal
accuracy, physical-owner dynamics, batching and vehicle behavior remain outside
this completed geometry/rate gate.
