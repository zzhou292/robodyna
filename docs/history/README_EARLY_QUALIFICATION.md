# Robo-dyna

Robo-dyna is a modular simulation project built from TL-FEA and Chrono, with
LS-DYNA-like CAE functionality as the long-term goal. The first acceptance case
is the original Yaris hitting a rigid triangle-mesh wall. Complete CUDA vehicle
dynamics is still under development; current qualified cases and limits are
listed in the [execution status](../planning/EXECUTION_STATUS.md).

The [active Yaris delivery plan](docs/YARIS_DELIVERY_PLAN.md) records the latest
live probes, module ownership, next implementation packages and promotion tests.
The [rendering architecture](docs/RENDERING_ARCHITECTURE.md) requires actual
deformation playback and a vehicle-crash video as part of delivery.

The complete prescribed elastic Q4 CUDA force gate now passes 29 new setup,
rotation, force and assembly checks. TL owns the small reusable operations;
`chrono/ReissnerShellSetup` copies actual Chrono rest data and its elastic
section matrix. Enable `ROBO_DYNA_ENABLE_SHELL_FORCE_CHECKS` against the qualified
coherent Chrono core to build these optional checks. This gate supplies forces
and energy for prescribed configurations. The next B1 gate adds 14 CUDA nodal
rotation/constraint checks, ten host shell mass/inertia checks and one accepted
Chrono output check. Coupled shell dynamics and graphical rendering remain open;
the [elastic coupon design](docs/ELASTIC_COUPON_DESIGN.md) defines the next case.

TL-FEA owns CUDA mechanics, shared state and stepping. Robo-dyna owns model/case
configuration, orchestration and results through Chrono infrastructure. The
canonical source directory is `robo-dyna/`; a legacy workspace path alias keeps
existing build trees and frozen evidence usable. New builds use this directory.

`chrono/AcceptedSurfaceMesh` connects accepted TL physical-node positions to
Chrono core's mesh and visual-shape objects. TL remains the dynamics owner. The
adapter preserves double coordinates and integer source identities, keeps
physical faces separate from their display triangles, and publishes a complete
validated frame atomically. Rejected/stale/wrong-owner frames preserve the last
published mesh. Mapped positions are copied; subsequent TL commits or destruction
do not invalidate the visible coordinates. Calls and rendering must be serialized.

The first binding is bounded to 4,096 display vertices and 8,192 triangles, with
caller-selected lower limits. Topology remains fixed. This is a small preview
integration gate; whole-vehicle capacity and VSG window/render behavior are
separate work. The adapter does not validate the model's mechanics or advance
physical time. Its mutable-shape path targets later VSG use; Irrlicht incremental
connectivity updates are not qualified by these headless checks.

From the workspace root, build the generic six-check test using the existing
Chrono core build and real TL nodal header, with no CUDA or Fortran requirement:

```sh
python3 Total-Lagrangian-FEA/tools/run_bounded.py --report crash-work/reports/app-snapshot-configure.json --cpus 2 --max-rss-gib 1 --timeout 90 -- cmake -S robo-dyna -B crash-work/build/robo-dyna-app-snapshot-rerun -DCRASH_ENABLE_CHRONO_SNAPSHOT_CHECK=ON -DChrono_DIR="$PWD/crash-work/build/chrono-core/cmake" -DCMAKE_BUILD_TYPE=RelWithDebInfo
python3 Total-Lagrangian-FEA/tools/run_bounded.py --report crash-work/reports/app-snapshot-build.json --cpus 2 --max-rss-gib 2 --timeout 120 -- cmake --build crash-work/build/robo-dyna-app-snapshot-rerun --target crash_chrono_snapshot_check --parallel 1
python3 Total-Lagrangian-FEA/tools/run_bounded.py --report crash-work/reports/app-snapshot-test.json --cpus 1 --max-rss-gib 0.5 --timeout 30 -- ctest --test-dir crash-work/build/robo-dyna-app-snapshot-rerun -R '^chrono_snapshot_integration$' --output-on-failure -j 1
```

The separate opt-in `CRASH_ENABLE_TL_CHRONO_CHECK=ON` composes TL's existing
qualification CMake targets and links `persistent_shell` only into an integration
test. Add `-DCMAKE_CUDA_COMPILER=/usr/local/cuda/bin/nvcc` and the actual compatible
`-DCMAKE_CUDA_ARCHITECTURES=120` for this workstation, then build target
`crash_tl_chrono_snapshot_check` with one job. Fortran remains optional; the bridge
needs neither native oracle. Run the four tiny tests through the same guard:

```sh
python3 Total-Lagrangian-FEA/tools/run_bounded.py --report crash-work/reports/app-tl-snapshot-test.json --cpus 2 --max-rss-gib 1 --timeout 60 --gpu 0 --max-gpu-growth-gib 1 -- ctest --test-dir crash-work/build/robo-dyna-app-snapshot-rerun -R '^tl_chrono_snapshot_integration$' --output-on-failure -j 1
```

These tests consume actual GPU shell `accepted()` states, keep source-owner
identity in the test coordinator, and verify that uncommitted/rejected trials
and another backend cannot publish a new frame. The numerical fixture remains
prescribed-motion qualification, with production rigid-motion behavior still
open. The reusable adapter has no qualification-header/library dependency.

## Force-driven TL nodal output

`chrono/NodalMeshOutput` binds the real TL `FENodalState` owner to the existing
Chrono accepted-mesh adapter. Four actual CUDA integration tests pass: additive
forces advance resident physical nodes; output cadence publishes only committed
positions; a late numerical overflow preserves the last visible frame and a
clean retry can publish; a different live owner cannot replace the bound mesh.
The optional rotational owner uses the same bridge; rejected spin leaves the
visible accepted frame unchanged. Positions remain binary64 and source IDs
remain integers above 2^53.

The app links TL's production `tl_explicit_nodal_state` target and Chrono core.
This path requires no shell-qualification library, Fortran, DEME or second
dynamics clock. State ownership and fixed-step advancement live in TL; this
adapter only reads accepted state at the application's output cadence. It
retains no source pointer between calls, and its host staging is preallocated.
The current owner admits at most 64 physical nodes, supplied isotropic
translational mass and optional world rotations with declared isotropic inertia.
This fixture uses the prescribed-constant-load rotational admission. Separate
elastic and QEPH history gates are tracked in the [execution status](../planning/EXECUTION_STATUS.md).
The fixture is a force-driven triangular display mesh; it is not an
elastic shell, mesh contact case, graphical viewer or Yaris crash.

From the workspace root, with the existing core-only Chrono build:

```sh
python3 Total-Lagrangian-FEA/tools/run_bounded.py --report crash-work/reports/nodal-output-configure-rerun.json --max-rss-gib 0.5 --timeout 45 -- cmake -S robo-dyna -B crash-work/build/robo-dyna-nodal-output-rerun -DCRASH_ENABLE_TL_NODAL_CHECK=ON -DChrono_DIR="$PWD/crash-work/build/chrono-core/cmake" -DCMAKE_CUDA_COMPILER=/usr/local/cuda/bin/nvcc -DCMAKE_CUDA_ARCHITECTURES=120 -DCMAKE_BUILD_TYPE=RelWithDebInfo
python3 Total-Lagrangian-FEA/tools/run_bounded.py --report crash-work/reports/nodal-output-build-rerun.json --max-rss-gib 2 --timeout 120 -- cmake --build crash-work/build/robo-dyna-nodal-output-rerun --target crash_tl_nodal_output_check --parallel 1
python3 Total-Lagrangian-FEA/tools/run_bounded.py --report crash-work/reports/nodal-output-tests-rerun.json --gpu 0 --max-rss-gib 1 --max-gpu-growth-gib 1 --timeout 45 -- ctest --test-dir crash-work/build/robo-dyna-nodal-output-rerun -R '^tl_nodal_output_integration$' --output-on-failure --parallel 1
```

All output/scene operations are externally serialized. Configure this optional
path explicitly; ordinary file-format and generic mesh tests stay independent
of CUDA. The state/stepper tests belong to TL's existing unit-test workflow.

## Finite Yaris mesh-wall normal-impact rig

`case/NormalImpactCase` now closes the force-driven CUDA loop against the actual
62-vertex/100-triangle canonical wall. A 0.1 m square mass patch spans the real
stitched seam. TL's `PlanarMeshContact` evaluates finite triangle coverage and
area-weighted normal forces; `FENodalState` owns accepted/trial positions,
velocities and time. The app checks prepared penetration before commit and
publishes accepted states through `NodalMeshOutput`. Chrono owns the output mesh
and existing archive/OBJ serialization. No Chrono dynamics system is stepped.

This is a contact/inertia rig, with supplied areal mass and no internal shell
elasticity. It admits fixed-footprint normal motion, zero friction/damping and
bounded penalty overlap. Nine TL contact tests and seven closed-loop app tests
pass on CUDA, covering independent force/work/impulse oracles, real seam
ownership, wall/patch refinement, finite-wall misses and failed-trial rollback.
The 10 canonical reader tests preserve binary64 coordinates and uint64 IDs.
This does not qualify oblique contact, CCD, folding, shells or a vehicle crash.

Configure this optional case against the already built Chrono core:

```sh
python3 Total-Lagrangian-FEA/tools/run_bounded.py --report crash-work/reports/normal-impact-configure-rerun.json --max-rss-gib 1 --timeout 45 -- cmake -S robo-dyna -B crash-work/build/robo-dyna-normal-impact-rerun -DCRASH_ENABLE_NORMAL_IMPACT=ON -DCRASH_CANONICAL_WALL="$PWD/crash-work/assets/yaris-wall/manifest.json" -DChrono_DIR="$PWD/crash-work/build/chrono-core/cmake" -DCMAKE_CUDA_COMPILER=/usr/local/cuda/bin/nvcc -DCMAKE_CUDA_ARCHITECTURES=120 -DCMAKE_BUILD_TYPE=RelWithDebInfo
python3 Total-Lagrangian-FEA/tools/run_bounded.py --report crash-work/reports/normal-impact-build-rerun.json --max-rss-gib 2 --timeout 180 -- cmake --build crash-work/build/robo-dyna-normal-impact-rerun --parallel 1
python3 Total-Lagrangian-FEA/tools/run_bounded.py --report crash-work/reports/normal-impact-test-rerun.json --gpu 0 --max-rss-gib 1 --max-gpu-growth-gib 1 --timeout 60 -- ctest --test-dir crash-work/build/robo-dyna-normal-impact-rerun --output-on-failure --parallel 1
mkdir -p crash-work/runs
python3 Total-Lagrangian-FEA/tools/run_bounded.py --report crash-work/reports/normal-impact-run-rerun.json --gpu 0 --max-rss-gib 1 --max-gpu-growth-gib 1 --timeout 60 -- crash-work/build/robo-dyna-normal-impact-rerun/case/robo-dyna crash-work/assets/yaris-wall/manifest.json crash-work/runs/normal-impact-rerun
```

The CLI requires a **new** output directory and verifies the exact canonical
manifest SHA256 with OpenSSL before parsing the same bytes. The case's count and
archive-metadata guard selects the intended model; the CLI provides byte-level
authentication. Its defaults run 140 fixed steps at 0.5 ms to 70 ms. The
accepted interval CSV labels force at `t_n` separately from state at `t_(n+1)`;
impulse and force work are logged once per successful commit. Existing Chrono
JSON archives preserve coordinate bits and connectivity on readback. OBJ files
use Chrono's visualization precision. The completed manifest is published only
after the requested horizon and artifact checks; an incomplete run cannot claim
success. Source wall friction is recorded but is outside this admitted law.

## Preserved unmodified Reissner reference: finite bending objectivity failed

`CRASH_ENABLE_CHRONO_REISSNER_CHECK=ON` builds seven headless GoogleTests against
the existing `ChElementShellReissner4`, with one Q4 and one centered elastic layer.
They prescribe current node positions/directors, capture the neutral reference
once, and evaluate internal forces directly. They never advance dynamics. The
option defaults OFF and adds no dependency to the accepted-surface adapter.

A separate Release core was built at `crash-work/build/chrono-fea-reference`
from Chrono revision `0166ac8c376d0b63e75547b3662d60eefe6896ee`, with
`CH_ENABLE_MODULE_FEA=ON`, FEA multiphysics and other optional modules OFF.
The original working `chrono-core` build remains separate. The one-job build
passed under the resource guard; its exact options are retained in the
[configuration record](../crash-work/reports/chrono-fea-reference-configuration.json).
No Chrono formulation source was changed or copied into the application.

**Observed result: 6/7 tests passed.** Neutral/reference retention, stress-free
finite rigid motion, analytic membrane traction/energy, small-curvature bending,
membrane-prestress superposition, and the unrotated mixed force/energy work check
passed. Finite rigid motion superposed on differential bending failed nine
metrics. The original [log](../crash-work/reports/reissner-reference-tests-1.log)
and [XML](../crash-work/reports/reissner-reference-tests-1.xml) retain all results:

| Failed metric | Measured error | Declared limit |
| --- | ---: | ---: |
| World-force covariance | 0.0244689 N | 5.41994e-7 N |
| World-couple covariance | 0.0216006 Nm | 9.62266e-7 Nm |
| Elastic-energy invariance | 1.01055e-7 J | 7.46603e-11 J |

Tolerances were fixed before execution: finite-transform comparisons use a
2e-8 relative term with separately declared absolute floors. The infinitesimal
bending and central-difference work checks have their own 2e-6 relative limits.
No tolerance was relaxed after the failure. Strain/curvature invariance and
post-rotation moment balance also failed; this is not a production oracle for
finite bending or a validation of the CUDA crash path.

The [source audit](../planning/ELEMENT_SELECTION.md) found a nonobjective
matrix-average/rotation-vector conversion and omitted variation of the
recomputed average frame in the force derivatives. Existing quaternion averaging
and polar decomposition are possible diagnostic references, but replacing only
the mean is not an established consistent force/tangent repair. Keep the six
passing scopes narrow. This baseline contains no correction. The subsequent
default-off consistent-force mode and its CPU/CUDA qualification are documented
in [the Chrono integration notes](chrono/README.md); the baseline build and
original failure reports remain intact.

To reproduce using the already built FEA core, from the workspace root:

```sh
python3 Total-Lagrangian-FEA/tools/run_bounded.py --report crash-work/reports/reissner-configure-rerun.json --lock crash-work/reports/workstation.lock --cpus 1 --max-rss-gib 0.5 --timeout 45 -- cmake -S robo-dyna -B crash-work/build/robo-dyna-reissner-baseline-regression-rerun -DCRASH_ENABLE_CHRONO_REISSNER_CHECK=ON -DChrono_DIR="$PWD/crash-work/build/chrono-fea-reference/cmake" -DCMAKE_BUILD_TYPE=RelWithDebInfo
python3 Total-Lagrangian-FEA/tools/run_bounded.py --report crash-work/reports/reissner-build-rerun.json --lock crash-work/reports/workstation.lock --cpus 1 --max-rss-gib 2 --timeout 120 -- cmake --build crash-work/build/robo-dyna-reissner-baseline-regression-rerun --target crash_chrono_reissner_reference_check --parallel 1
python3 Total-Lagrangian-FEA/tools/run_bounded.py --report crash-work/reports/reissner-tests-rerun.json --lock crash-work/reports/workstation.lock --cpus 1 --max-rss-gib 0.5 --timeout 45 -- crash-work/build/robo-dyna-reissner-baseline-regression-rerun/chrono/crash_chrono_reissner_reference_check --gtest_output=xml:crash-work/reports/reissner-tests-rerun.xml
```

The unchanged test returns failure; it does not skip or mark the failed physical
gate as an expected pass. CTest registers it as `chrono_reissner_reference`.
The default application tests remain independent of this opt-in experiment.

## Original wall preparation

The first implemented slice compiles the **original Yaris coarse V1l wall** into
an indexed fixed triangle mesh. It is a scoped wall preparation tool, not a
vehicle importer, solver, or general LS-DYNA parser. It uses Python's standard
library, runs on one CPU, and does not use the GPU or download anything.

The original geometry has 62 nodes and 46 quads. The compiler inserts the eight
existing lower-grid seam nodes into the upper quad's long boundary edge, then
chooses nondegenerate convex fans. The result has **62 vertices, 100 triangles,
161 edges and 22 boundary edges**, with normals facing −X. No vertices are moved
or duplicated during stitching. It reads the original −4550 mm include
translation and converts millimetres to metres, placing the mesh at **X=0.05 m**.

Original node/quad IDs and the include's assembled IDs (10,000,000 offset) are
retained. The output manifest maps every triangle back to its source quad and
every vertex back to its source node. It also separates whole-wall reactions
from the original lower-region segment set, which excludes the upper quad.

The wall is fixed, with zero additional collision thickness and no dynamic DOFs.
The original 1 mm display-shell thickness is recorded separately. Vehicle shell
half-thickness still belongs to the future contact adapter. The source finite
plane cards provide documented settings only; they do not generate duplicate
contact forces. Their rectangular bounds differ slightly from the source mesh;
both are recorded and the mesh is authoritative.

The two small original wall/setup input files are retained with their notices
in `tests/data/yaris-wall/`. From the workspace root, regenerate with:

```sh
python3 Total-Lagrangian-FEA/tools/run_bounded.py \
  --report crash-work/reports/robo-dyna-wall-import-rerun.json \
  --lock crash-work/reports/workstation.lock --cpus 1 --max-rss-gib 0.25 --timeout 30 -- \
  python3 robo-dyna/tools/import_yaris_wall.py \
  --source-model robo-dyna/tests/data/yaris-wall \
  --output crash-work/assets/robo-dyna-wall-regenerated
```

The two input files are SHA256-pinned to the inspected originals; the tool fails
on another revision rather than silently changing the scenario. The prepared
`source/` snapshots also support regeneration without the temporary download
folder. Input files are preserved byte-for-byte, including their notices.

Outputs in `crash-work/assets/yaris-wall/`:

- `wall.obj`: the actual SI triangle mesh for collision and rendering.
- `manifest.json`: source mappings, units/translation, topology checks, contact
  settings, reaction regions, source hashes and mesh hash.
- `source/wall.key`, `source/combine.key`: exact original input snapshots.
- `SHA256SUMS`: checksums of those four deterministic artifacts. Reports from
  resource monitoring are separate and are not part of reproducible geometry.

Run the small independent geometry/import tests from their pinned fixtures:

```sh
python3 Total-Lagrangian-FEA/tools/run_bounded.py \
  --report crash-work/assets/yaris-wall/tests-run.json \
  --lock crash-work/reports/workstation.lock --cpus 1 --max-rss-gib 0.25 --timeout 30 -- \
  python3 -m unittest discover -s robo-dyna/tests -p 'test_yaris_wall.py' -v
```

The preserved canonical wall remains in `crash-work/assets/yaris-wall`. New
imports record the renamed generator and its current source hash; regenerate
into the separate directory above to preserve the historical manifest.

The same tests are registered with CMake/CTest:

```sh
cmake -S robo-dyna -B crash-work/build/robo-dyna
ctest --test-dir crash-work/build/robo-dyna --output-on-failure --parallel 1
```

Use the shared `run_bounded.py` wrapper for these commands during coordinated
workstation execution as well.

The tests check independent polygon area, all face orientations, conforming seam
incidence, topology, source coverage, SI coordinates, OBJ/manifest agreement,
repeatability and fail-closed malformed/missing inputs. Passing them qualifies
wall preparation only; mesh contact, crossing protection and structural crash
simulation remain separate gates.

Model attribution: the source model was developed by the Center for Collision
Safety and Analysis at George Mason University under an FHWA contract. The
original attribution and usage notice remain in the preserved `combine.key`.

## Original vehicle geometry and blocking inventory

`tools/import_yaris_vehicle.py` compiles the original vehicle include from the
already downloaded coarse V1l archive. It preserves **393,165 nodes, 358,457
shells, 15,234 solids and 4,685 beams**, with 919 part definitions. Coordinates
are converted from millimetres to metres in the **original untransformed vehicle
frame**. The wall/setup includes are inventoried separately; they are not
assembled into this geometry or treated as implemented mechanics.

The compiler streams the pinned archive with Python's standard library. It
reuses the wall tool's validation/provenance utilities and the prior model
inventory/census. It preserves raw source connectivity, node/element order,
source IDs, source line numbers, repeated triangle/wedge slots, beam
orientation/release fields and blank-field masks. Compact node-index arrays
provide a direct route to later device upload. It does not convert the model
into T10 tetrahedra or infer a material/element formulation from the mesh shape.
The source's blank coordinates for node 2000001 map explicitly to zero; the
manifest records that mapping. Node constraint codes are retained but not
applied. Beam n3/local orientation semantics remain unresolved.

Every keyword block across all four source decks receives a source-line range,
hash and disposition. Geometry and identity metadata are distinguished from
**BLOCKING** material, section, contact, connection, mass, cavity, loading,
initialization and output semantics. The manifest sets `simulation_ready=false`
and `m1_complete=false`. An explicit admission check prevents this geometry-only
artifact being accepted as a runnable crash model. A keyword inventory is not
field-level capability coverage.

The existing OpenRadioss release has a reader shared object and schema files,
but the inspected installation does not provide an isolated reader SDK with a
verified callable geometry API. The selected source converter depends on SDI
model/entity interfaces; most reader API/build blobs are absent from the local
sparse checkout. The previous probe's converted VTK already reflects complete
case assembly and solver mappings. None provides a verified direct route to
preserving the original records within this bounded slice. This is a local
integration finding, not a claim that OpenRadioss cannot expose a reader. Its
reader remains a reference for the later complete semantic import.

From the workspace root, prepare the canonical arrays without running a solver:

```sh
python3 Total-Lagrangian-FEA/tools/run_bounded.py \
  --report crash-work/reports/yaris-vehicle-import.json \
  --lock crash-work/reports/workstation.lock --cpus 1 --max-rss-gib 1 --timeout 120 -- \
  python3 -B robo-dyna/tools/import_yaris_vehicle.py \
  --source-archive /tmp/chrono-yaris-plan/2010-toyota-yaris-coarse-v1l.zip \
  --output crash-work/assets/yaris-vehicle
```

The output contains little-endian raw arrays with exact dtype, shape, field names
and SHA256 checksums in `manifest.json`. Source IDs remain alongside compact
indices; beam compact connectivity covers n1/n2 only. `source_model.zip` retains
the original compressed archive outside git, including notices and all currently
blocking cards. `SHA256SUMS` covers the arrays, manifest and archive. The staged
archive can be supplied as `--source-archive` for later regeneration. Adding
`--require-simulation-ready` intentionally fails admission.

Run the tiny fixture tests with `test_vehicle_geometry.py`, or run the full
prepared-artifact test with `test_yaris_vehicle_model.py`, using the same bounded
wrapper. The full test compares **every** node and structural connectivity record
against the original archive, checks source lines and index round trips, and
verifies all checksums. It requires the prepared asset directory and never
silently skips missing data. Set `YARIS_VEHICLE_ASSET_DIR` to override the default
workspace asset location.

CMake always registers the small fixture suite. Configure with
`-DCRASH_YARIS_VEHICLE_ASSETS=/absolute/path/to/crash-work/assets/yaris-vehicle`
to additionally register the full artifact gate. Keep CTest serialized and under
the shared workstation guard.

## External Chrono wall integration check

The optional `chrono/` application follows Chrono's external CMake template and
links its existing core library. It loads the prepared OBJ through
`ChTriangleMeshConnected`, checks all 100 triangle normals, area, vertex count and
SI wall position, and attaches that mesh to a fixed visual body. It does not
advance Chrono dynamics or evaluate contact. The checked-out OBJ loader reads
coordinates through `float`; the test allows that input roundoff and reports it.
CUDA collision must use the canonical double-precision asset, not round-tripped
visual coordinates.

A bounded core build was verified with demos, tests, optional FEA/robot modules,
OpenMP, SIMD and YAML disabled. The local Chrono package fix ensures disabling
robot models does not make external `find_package` require an absent library.
No system installation is needed; the build tree can be linked directly.

After the core build and wall preparation, configure the application using:

```sh
python3 Total-Lagrangian-FEA/tools/run_bounded.py \
  --report crash-work/reports/app-configure.json -- \
  cmake -S robo-dyna -B crash-work/build/robo-dyna \
  -DCRASH_ENABLE_CHRONO_CHECK=ON \
  -DChrono_DIR="$PWD/crash-work/build/chrono-core/cmake" \
  -DCRASH_WALL_OBJ="$PWD/crash-work/assets/yaris-wall/wall.obj" \
  -DCRASH_YARIS_VEHICLE_ASSETS="$PWD/crash-work/assets/yaris-vehicle"
```

Build with `cmake --build crash-work/build/robo-dyna --parallel 1` and run CTest
with `--parallel 1`, both through the same resource guard. The test is named
`chrono_wall_integration`. It verifies mesh/scene data and external library
linkage, not a graphical window or vehicle crash.
