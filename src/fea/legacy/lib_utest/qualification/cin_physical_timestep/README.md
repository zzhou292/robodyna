# Physical CIN structural timestep screen

The new admission is opt-in:

```cpp
admission.structural = {
    NodalCinStructuralProfile::NativeOrdinaryRigidTrace, 0.8};
auto result = AdvanceStaggeredCin(owner, token, admission);
```

The existing token, complete assembly, CIN witness and no-release qualification
are still required. Default admission remains unchanged. An enabled operation
returns its physical minimum in `NodalReport::stable_dt`, on success or a
structural `StepTooLarge` rejection. It checks the full fixed drift duration,
including the first interval with a half kick. Accepted state is unchanged on
rejection; discard and retry use the normal sole-owner transaction.

The screen runs after actual CIN M/J/STI/STIR transfer, before any inverse or
motion update. It consumes raw private coefficients; it never reconstructs mass
from inverse values. Ordinary free translation/rotation uses native DTNODA
`factor*sqrt(2*coefficient/stiffness)`. Zero stiffness contributes no bound;
absent ordinary rotation remains explicit and cannot carry positive STIR.

PART/plain body members use actual aggregate mass and principal inertia and the
qualified current force frame. For the local isotropic scalar surrogate
`D_i=diag(kN_i*I3,kR_i*I3)`, rigid motion projects member displacement as
`u_i=t+theta cross r_i`. The rounded-up trace of the mass-normalized 6x6 projected
operator bounds its largest eigenvalue. The admitted body bound is
`factor*sqrt(2/trace_upper)` with downward-rounded final operations. This is
conservative relative to the frozen undamped oscillator bound. Individual
member M/J—including authentic zeros—never enters the body response.

This analytical local surrogate screen is not a proof for the complete evolving
nonlinear tangent, gyroscopic terms, damping, contact or joints. The caller must
retain the other selected admission mechanisms. There is no mass scaling, new
clock, coefficient mutation, retained array, or control/arena size change.
The body helper reuses existing `RigidNormalResponse` and outward-rounded
positive arithmetic. No force, material, CIN transfer or rigid recurrence
expression changed.

## Donor receipt and limitation

The manifest pins complete OpenRadioss donors at
`a62b27e6baa555d222a580d6218867d0be4d70b5`. The independent native oracle extracts
exact DTNODA translation lines256-260 and rotation468-472, plus RGBODFP119-121
per-member plain-body stiffness terms. Full source identities and exact fragment
hashes are checked before compilation. No hand-mirrored native formula or body
repair is compiled.

The rigid-material branch is explicitly **not** an unmodified native oracle:
DTNODA358-378 retains the last MASS/STI for all collected body indices;
568-583 uses stale nodal N and overwrites the body result with nodal fields.
Complete RMATFORP -> RMATPON does not use its STIFN/STIFR arguments or populate
RBYM27/28. Native plain RGBODFP does accumulate kN and kR+|r|²kN; INIRBY839-840
uses aggregate mass/min principal J. These observations are retained separately
from the chosen analytical body trace. The next joint TT0 main-coefficient
summary will use an explicitly named physical policy and its own authenticated
owner query, not the trace as a fictitious native main stiffness.

The detailed caller and joint ordering audit is in the workspace planning file
`planning/CIN_PHYSICAL_TIMESTEP.md`. Source caches are reused in place; no
runtime source/network dependency is introduced.

## Owning gates

Configure this directory with the usual GNU Fortran/CUDA120 environment:

```
-DTL_CIN_PHYSICAL_TIMESTEP_NATIVE=ON
-DTL_CIN_PHYSICAL_TIMESTEP_CUDA=ON
-DTL_CIN_PHYSICAL_TIMESTEP_MATRIX=ON
-DTL_CIN_TIMESTEP_SOURCE_CACHE=/home/jsonzhou/Desktop/chrono-work/crash-work/deps
```

Targets/CTest selectors:

- `cin_physical_timestep_host`: three bounded value functions.
- `cin_physical_timestep_matrix`: independent dense 6x6 eigenvalue, PSD and
  dropped-lever/torque controls (one function).
- `cin_physical_timestep_native`: two native positive-vector/per-member tests.
- `cin_physical_timestep_source_identity`: complete donor/extraction check.
- `cin_physical_timestep_cuda`: two actual combined owner tests. Loaded
  PART/plain/ordinary/CIN state, first-half-kick/later-full-kick, late plain and
  authentic zero-M/J PART member limits, current transferred secondary
  stiffness, ordinary rotation, accepted nodes/groups/CIN history preservation,
  exact retry, default-off and invalid-profile controls.

Affected existing gates: `tied_cin_runtime_host/native/cuda`,
`rigid_assembly_owner_host/cuda` and its native identity, plus the current
`physical_common_publication` owner test configuration. Keep its generic-row
stiffness rejection. Owning Bazel targets are
`//lib_src/solvers:cin_structural_step_values`,
`//lib_src/solvers:explicit_nodal_state`, and
`//lib_utest/qualification/cin_physical_timestep:{cin_physical_timestep_host,cin_physical_timestep_cuda}`.
The existing rigid owner fixture target exposes only its test header to this
new qualifier; production storage is unchanged.

## Author evidence boundary

Three host functions and four source/CUDA-shaped C++ syntax units passed under
1 CPU/512 MiB. Complete donor identity/extraction passed. No Fortran, native
runtime, NVCC, GPU or full-source run was performed by the author.

Both Eigen oracle compilation attempts exceeded 512 MiB; reports remain intact.
The independent matrix target remains default ON for the root's larger owning
gate; author configured that separate target OFF solely for the tiny host
build. The original underflow control used denorm_min times1, which correctly
remained positive; the corrected test uses denorm_min times0.5. No production
arithmetic or tolerance changed to make the host test pass. The first owner
syntax pass exposed a test-only range-view iteration mismatch, corrected to
explicit count/index traversal. Reports are in
`crash-work/reports/cin-physical-timestep-author-1`.
