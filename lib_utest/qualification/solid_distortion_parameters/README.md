# Eight-node distortion parameter gate

Source candidate from e656; no vehicle profile, force kernel or accepted history
is changed. `materials/law42/MechanicalSlots.h` is reusable material preparation;
`solid_common/distortion/Parameters.h` is the small SDISTOR_INI/SCRE_SIG3 leaf.

Admission is SI, active eight-node H24 LAW42 alpha2/no-Prony, prepared material,
0 <= nu <= double(.48999f), OFF=OFFG=1, ISMSTR10. Input SIG is the full six-component
native Cauchy stress in the caller frame; it excludes QVIS. No principal-stress
solver or deviatoric projection belongs in this parameter path. Output only
contains LL, FLD (N*s/m^2), STI_C, MU, FQMAX and ISTAB. EINT_DISTOR remains a separate future
energy channel; this module does not alter it, STI, LAW90 STIN, force or history.
S6 geometric distortion stays disabled. Invalid/unsupported/overflow inputs leave
all output fields unchanged. Broader activity/ISMSTR12/high-nu branches are rejected.

Mechanical slots follow authenticated reader/update order: PM32=GS, PM100=reader
bulk, PM107=2*max(PM32,PM100). Prepared material consistency reuses LAW42 Prepare;
it does not invent density/cutoff. Named ONEP333 preserves its WP expression, and
bare .4/.48999 thresholds are widened binary32 constants. SCRE_SIG3 takes the
minimum of all six stored components, including signed shear, using strict `<`.

The test-only independent oracle is the existing qualified checkout at
94fc2db3b3de4d04509d5f5908c02844eb305775. Configure this directory with
`TL_DISTORTION_ORACLE_ROOT` pointing to that checkout. Its native source manifest
and exact a62 bytes are verified, then its existing CMake native targets and
parameter probe are reused. FQMAX is source-pinned to EP02; the existing probe
does not observe it and we do not claim otherwise. No new Fortran arithmetic or
wrapper is copied. Native CPU tests cover slots, material/kinematic inputs,
float thresholds, full hydrostatic stress, all six strict component thresholds,
signed-shear differences from principal stress, damping floor/cap, and atomic
rejection/repair. Optional `TL_DISTORTION_PARAMETERS_CUDA=ON` compares actual GPU
values with the same native probe and checks repeated results and mixed failures.

Builds/GPU execution require the workspace lane and bounded runner. Source-only
checks are not numerical qualification. Geometry/damping-force/face/corner
composition, complete H24 adapter, S6 hourglass adapter, LAW90 caller, native work,
owner integration, forecasts and fresh timestep admission remain separate gates.
