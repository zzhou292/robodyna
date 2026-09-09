# Crash application preparation

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
  --report crash-work/assets/yaris-wall/import-run.json \
  --lock crash-work/reports/workstation.lock --cpus 1 --max-rss-gib 0.25 --timeout 30 -- \
  python3 crash-app/tools/import_yaris_wall.py \
  --source-model crash-app/tests/data/yaris-wall \
  --output crash-work/assets/yaris-wall
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
  python3 -m unittest discover -s crash-app/tests -p 'test_yaris_wall.py' -v
```

The same tests are registered with CMake/CTest:

```sh
cmake -S crash-app -B crash-work/build/crash-app
ctest --test-dir crash-work/build/crash-app --output-on-failure --parallel 1
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
  python3 -B crash-app/tools/import_yaris_vehicle.py \
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
  cmake -S crash-app -B crash-work/build/crash-app \
  -DCRASH_ENABLE_CHRONO_CHECK=ON \
  -DChrono_DIR="$PWD/crash-work/build/chrono-core/cmake" \
  -DCRASH_WALL_OBJ="$PWD/crash-work/assets/yaris-wall/wall.obj" \
  -DCRASH_YARIS_VEHICLE_ASSETS="$PWD/crash-work/assets/yaris-vehicle"
```

Build with `cmake --build crash-work/build/crash-app --parallel 2` and run CTest
with `--parallel 1`, both through the same resource guard. The test is named
`chrono_wall_integration`. It verifies mesh/scene data and external library
linkage, not a graphical window or vehicle crash.
