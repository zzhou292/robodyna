# Explicit global LAW1 coefficient thickness

Source-only first numerical slice. Native NPT0 global integration reuses the
existing QEPH/T3 analytic force, ordinary shell history, stabilization, STI and
force projection. No constitutive solver or material-point history was added.
Legacy `EvaluateForce` keeps reference coefficient thickness and its former
failure/result contract. A source proof pins both numerical bodies after the
single coefficient-input routing substitution; historical oracle manifests stay
unchanged.

`EvaluateGlobalLaw1Force(profile, reference, history, interval, output)` uses a
caller-supplied `ShellGlobalLaw1Profile`. `Reference` selects reference thickness
(ITHK0); `Accepted` selects prior accepted thickness (ITHK1). QEPH applies the
pinned CNCOEF3B MYREAL8 EM20 floor, explicitly converted by
`coefficient_working_length_m`; T3's selected C3COEF3 branch has no such floor.
The profile's length is independent of `projection_working_length_m`. Physical
inputs/outputs remain SI. Reference/history layout is unchanged; this value API
does not authenticate a source profile or prevent a caller changing it between
calls. Immutable catalog/execution binding is the next distinct slice.

Native context: centered LAW1, IGTYP1, explicit ISMDISP0, T3 ISMSTR-1,
QEPH ISROT0/IDRIL0 and DM=DN.015. The existing positive reported-thickness and
finite geometry/arithmetic domain remains. There is no claim that all native
absolute numerical guards become unit-invariant. Metre/mm/tonne conversion tests
cover the new coefficient boundary and selected full force/history paths; QEPH
uses its separately qualified projection metric. Floor-neighbor checks allow
only the explicit input/output unit-rounding bound of two ULPs, not a broad
force tolerance.

The independent native adapter exposes the real ITHK argument while retaining
complete CNCOEF3B/C3COEF3 and existing global force leaves. Tests compare both
thickness modes over loaded, rotating, held and reversed prescribed paths;
check all native force/history/STI channels, exact host/device parity, legacy
results, invalid policy/phase and late coefficient-failure preservation/retry.
The NPT3 oracle is retained as a separate comparison: global ONE_OVER_12 and
layered promoted REAL4 .0833333 moment weights are distinct, and layer history
is never synthesized from global resultants. The measured differences are XML
properties, not an assumed equivalence or whole-model verdict.

Configure this directory with `-DTL_GLOBAL_LAW1_CUDA=ON`. Build under the owning
resource guard, run non-CUDA and actual CUDA CTests separately, then Bazel
`:consumer` and `:source`. CMake includes the unchanged global native and layered
LAW1/J2 regression targets. The consumer links no GTest/native Fortran reference.
