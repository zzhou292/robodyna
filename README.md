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
