# Coherent optional FEA multiphysics profile

Status: all four original demos compile and all five CPU material/geometry
admission cases pass (`robodyna-field-fea-build-2.json`). Interactive field
simulations and the inherited contact limitations below remain separate.

## Build ownership

The shared configuration is generated in `src/core/configuration`. The optional
source owner is declared by `build_defs/chrono/fea_multiphysics.bzl` and selected
by the mixed native aggregate. `SOURCES.json` pins the 38 original implementation
files and their headers. `ChDrawer.h` required three direct visualization
includes for self-contained compilation; the exact inverse is recorded in
`docs/migration/SOURCE_TRANSFORMATIONS.json`. Numerical code is unchanged.

`//build_defs/features:fea_multiphysics` defaults to false. The
`--config=multiphysics` spelling enables this build-wide setting. It controls the
one generated `ChConfig.h` used by the entire configured native dependency graph,
plus the native aggregate's dependency on the one field-source owner. Default
header contents and existing compiler arithmetic flags are unchanged. The38-TU
owner depends on the existing aggregate headers rather than the aggregate
implementation, avoiding a target cycle with the existing contact CPPs. It uses
the same inherited PCH include order and target-local host compilation flags.
There is still one core in each binary. Different configured build batches may
have different artifacts; do not combine them manually.

Canonical public target: `//src/fea/multiphysics:fields`. Its mixed backend
relationship is explicit; this is not independent FEA/MBD domain linkage. The
four direct original demo mains use Irrlicht and Pardiso without source changes.
Their unused VSG selector variable does not determine the instantiated renderer.

`//examples/fea:native_demos` remains the29-program baseline slice.
`//examples/fea:multiphysics_demos` contains all four optional programs.
`//examples/fea:all_demos` contains all33 programs and deliberately fails
admission if the required profile is absent. No `select` silently drops a program
from an aggregate claiming complete coverage.

Existing foreign CMake and SWIG/native-binding targets are marked incompatible
under this profile. Their current outputs and historical parser/runtime evidence
are baseline-only. Supporting their optional layout requires configuring CMake
and SWIG coherently and running their own ABI/runtime gates; generating baseline
wrappers and linking an optional core is not accepted by this proposal. Other
future ABI flags such as OpenMP/Multicore should use additional typed settings
and compose with this one in the shared configuration, not inject per-demo flags.

## Qualification sequence (root owns the workstation guard)

1. Baseline configuration: compare generated `ChConfig.h` bytes with the currently
   qualified baseline; preserve native compile flags and source ownership counts.
   Run the existing API/mesh/contact/source tests under the default profile.
2. Optional configuration: inspect actual compile owners (38 new originals once,
   same existing contact CPPs once); no duplicated core or header configuration.
3. Build and run `bazel test --config=multiphysics
   //tests/fea_multiphysics:field_admission_test` under the CPU guard. Five cases
   exercise real Fourier flux/heat capacity, nonlinear-strain stress/tangent,
   field node/DOF ownership, affine hexahedral interpolation and a live field
   contact proxy alongside an existing mechanical node. No GUI/GPU is launched.
4. Compile `bazel build --config=multiphysics
   //examples/fea:multiphysics_demos`; then `//examples/fea:all_demos` for full
   FEA coverage. Actual Irrlicht and Intel2023 SDKs are required. These programs
   are not automatically executed: runtime path/output/solver/graphics admission
   remains separate.
5. Reject explicit baseline-only bridge or binding targets under the optional
   profile. Their inability to mix profiles is intentional and must be visible.
6. The all-demo build matrix includes both the baseline and optional batches.
   Do not claim a default-only build compiled all four optional programs.

## Inherited contact limits: keep visible and unfixed in this build batch

- `ChContactSurfaceNodeCloud.h:311` allocates
  `ChConstraintTuple_1vars<3>` for a field contact. Its implementation at
  `ChContactSurfaceNodeCloud.cpp:142` statically casts those tuples to
  `ChConstraintTuple_1vars<6>`. A focused sanitizer-enabled field Jacobian
  reproducer should establish the failure before any separate source repair.
  The current admission test intentionally does not call this unsafe path and
  does not qualify field-contact dynamics.
- `ChContactSurfaceMesh.h:697` sums the legacy XYZ/XYZRot lists in
  `GetNumTriangles()` and omits `m_faces_field`. A regression reproducer should
  build one field hexahedron, extract its12 boundary triangles using
  `OutputSimpleMesh`, then compare that count with the public count. Existing
  count semantics are not silently redefined by a build migration.
- `ChContactSurfaceNodeCloud.h:461` and `ChContactSurfaceMesh.h:779` add vectors
  to existing class layouts under the feature macro. The corresponding CPPs
  add constructor/geometry/sync behavior. This is why a feature-only define on
  demo/new-owner translation units would be an ODR/ABI defect.

The five passing tests are limited module admission, not a long simulation,
accuracy comparison, optional contact acceptance or CUDA implementation claim.
