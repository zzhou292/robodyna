# QEPH native reference: Q1 startup and prescribed kinematics

This explicit opt-in host library calls complete pinned OpenRadioss routines.
It is a one-element qualification oracle, without a solver process, CUDA state,
time integration, stresses, material history or force production. Q2 must add
the coherent native LAW1/stabilization/history path before a force port is judged.
The native library compiles. First execution passed 10 of 11 tests: static geometry,
covariance, affine rates and the analytic Z-spin case passed; the proposed
all-component second-order general rigid-rate gate failed. Its source/binary
and results remain checkpointed. The corrected exact planar analytic reference
now passes all 11 functions at the original arithmetic/covariance tolerances;
`crash-work/reports/qeph-q1-tests-2.xml` retains all rigid-path components.
Warped temporal order remains unqualified. This is not a dynamics accuracy
promotion. See workspace
`planning/QEPH_Q1_TEMPORAL_FINDINGS.md` for all-component formulas and evidence.

The public value API is `QephReference.h`. `Reference` holds validated immutable
startup data. `PrescribedInterval` provides endpoint positions and midpoint
world translational/angular velocities; its time/index are caller provenance,
not an internal clock. Every operation stages outputs and preserves all caller
bytes on rejection. A successful Initialize may replace an existing Reference.

The selected source branch is centered uniform isotropic QEPH, IREP=0,
ISMSTR=-1, ITHK=0, ISROT=0, IVECTOR=0, IRESP=2, explicit IMPL_S=0. The actual
section defaults are declared in the input; Q1 does not infer a deck or admit
plasticity, damping or a physical trajectory. Geometry must have a conditioned
convex projection. Native frame deactivation, nonfinite projection or arithmetic
results reject publication; no coordinate flattening or repair is performed.

## Exact source reuse

`source-manifest.json` pins 27 original files to OpenRadioss commit
`a62b27e6baa555d222a580d6218867d0be4d70b5`, with SHA256 and upstream Git blob
identities. `verify_sources.py` checks all originals and the exact inclusive
routine ranges of nine extracted compilation units. Full original notices and
AGPL-3.0-or-later license are retained under `original/`.
`prepare_sources.py` changes only four complete module-name tokens in build-tree
copies, making their private identities visible to CMake's dependency scanner.
The build's `prepared/prepared-sources.json` records every input/output SHA256;
the build verifies prepared bytes again. Originals and exact extracts remain
unchanged. This addresses a measured CMake module-copy failure, without changing
native expressions or adding aliases to unprefixed module files.

* CNEVECI and the starter CLSKEW3 produce startup axes and projected area;
  CDERII supplies unnormalized derivatives and projected local coordinates.
  CCOORI's global indexed gather is unnecessary for an already packed cell.
* The uniform centered CINMAS branch is adapted in NativeQephStartup.F with
  the source expression order: mass `rho*t*A/4`, total nodal rotary inertia
  `m*(A/12+t*t*(1/12+0*0))`. Physical and added partitions are reported
  separately; their sum is not substituted for the native total expression.
* CZCORC1, engine CLSKEW3, real CORTDIR3, full CZCORP5 and CZDEF execute the
  current geometry, second-order velocity correction, warpage projection and
  eight regular/six hourglass rates. Complete inverse helpers and CZCORP5X are
  retained for link closure; the wrapper never selects XFEM or implicit paths.
* Both CLSKEW3 symbols are distinct. Native routines and COMMON blocks receive
  private compile-time prefixes; real module names are privately prepared. A mutex
  serializes this library's explicit context. No no-op replacement routines,
  external solver symbols or native allocators are linked.

Native area is the mean-plane projected area, not the true warped bilinear
surface area. Raw warpage and the source planar switch are separately visible.
CZCORP5 keeps squared warpage locally; the bridge reproduces CZCORC1's exact
pre-projection centroid/dot expression for the raw diagnostic only. This value
does not feed back into native geometry or rates.
Flat-branch normals are explicitly +Z and unused projection arrays zero.
Regular and hourglass component order/units are stated in the public types.
The rates retain the native endpoint-X/midpoint-V temporal convention. For a
general rigid axis this path has an analytically explained O(h) transverse
shear-rate residual; the world-Z special case has a cubic residual. Static
objectivity and faithful arithmetic do not certify temporal accuracy, bounded
accumulated rigid-motion work, or a dynamics timestep.

## Bounded standalone build

Use the workstation guard externally; this project starts no resource jobs.
From the workspace, configure with:

```sh
cmake -S Total-Lagrangian-FEA/lib_utest/qualification/native/qeph \
  -B crash-work/build/qeph-q1 -DCMAKE_BUILD_TYPE=Release -DQEPH_Q1_BUILD_TESTS=ON \
  -DCMAKE_Fortran_COMPILER=/home/jsonzhou/Desktop/chrono-work/crash-work/tools/gfortran-11.4.0/gfortran-local
cmake --build crash-work/build/qeph-q1 --parallel 1
ctest --test-dir crash-work/build/qeph-q1 --output-on-failure
```

GNU Fortran, a C++17 compiler, Python3 and an existing GTest installation are
required; supply the existing GTest prefix if it is not discoverable.
The example selects the already retained GNU 11.4.0 workspace compiler wrapper;
it does not install or replace a system compiler.
Target `qeph_q1_native` is a static reference library; `qeph_kinematics_check` is the
small CPU-only GTest executable. No parent CMake or production solver is edited.
Existing native precision/includes/compiler controls are reused, with bounds
checks, no fast math and no contraction. The arithmetic/covariance tolerances
were frozen before first execution. Tests cover analytic geometry/mass, full
warped projection, planar rates, covariance, rigid-path diagnostics and failure
publication. The corrected order characterization and original failure are
documented above.
