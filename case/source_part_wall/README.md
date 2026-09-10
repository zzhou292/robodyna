# Source-part mesh-wall geometry

This host preparation slice reuses the existing contact reference operations for
all 117 original Yaris part nodes and all 94 physical shell parents. It supplies
geometry and source identity for the next impact experiment. It does not admit
impact dynamics, choose penalty stiffness, construct structural mass or launch
CUDA work.

`SourcePartContactGeometry` owns the original Q4 parametric references, native T3
material measures and `NodalWallWeights`. The existing owning area arithmetic is
unchanged: Q4 contact shares use the center-area reference measure divided among
four nodes, and T3 shares use native area divided among three. Both source-parent
to sorted-weight and inverse maps are explicit. The original qualification
`nodal::PreparedSource` delegates this geometry preparation while retaining its
existing `FixtureMass` policy. The geometry component never exposes that proxy;
future dynamics must supply actual `ShellBatchBinding` mass and owner DOFs.

`PlacedCanonicalWall` uses the existing authenticated original wall-copy branch
of `WallTessellation`, then applies one caller-declared X translation to a new
copy. Canonical loading stays fixed at X=.05 m, with all 62 vertices, 100
triangles and 46 source quads retained. Y/Z coordinates, winding, connectivity,
node IDs and triangle/source-parent IDs remain unchanged. The declared shift and
its binary64 representation are distinct from the represented placed wall X.
No source manifest is rewritten.

`CheckWallCoverage` checks a caller-declared world motion box containing the
whole original part. It explicitly projects both box X coordinates onto the
placed wall X and delegates finite mesh/exposed-boundary coverage to
`CheckPlanarWallBox`. A successful setup query requires future state guards to
keep the actual trajectory in that declared box; it is not a dynamics proof.

`WritePlacedCanonicalWallArtifacts` reuses `MeshArchive` for the actual placed
mesh JSON/OBJ and writes the original canonical manifest under a distinct name.
Separate placement metadata binds source hashes, declared shift bits, actual
plane bits, placed mesh hashes and complete source ID mappings. The caller owns
the run inventory and completion marker. Existing canonical replay schemas are
not reused for a translated wall.

Five host test functions cover all original source parents and inverse maps,
legacy area parity, the distinction between native structural mass and proxy
mass, preparation/placement failure preservation, exact placed source identity,
actual Chrono mesh archive roundtrip, and finite-wall coverage with both X
endpoints correctly projected. Enable the optional target with:

```
-DROBO_DYNA_ENABLE_SOURCE_CONTACT_CHECKS=ON
-DROBO_DYNA_SOURCE_SHELL_COLLECTION=ON
-DROBO_DYNA_SOURCE_PART_WALL_GEOMETRY=ON
-DROBO_DYNA_SOURCE_PART_READINESS=/absolute/pinned/readiness.json
-DCRASH_CANONICAL_WALL=/absolute/pinned/wall/manifest.json
```

Owning executable: `robo_dyna_source_part_wall_geometry_check`.
CTest name: `source_part_wall_geometry`. This feature requires no CUDA build or
native Fortran oracle. All five functions pass, along with the retained source
geometry, force and CUDA nodal-wall regressions. Evidence is retained in
`crash-work/reports/moving-startup-geometry-tests-1.ctest.log` and
`source-wall-geometry-regressions-1.ctest.log`. The old/new wrapper comparison is
adapter parity; the retained source-area tests provide independent numerical
checks. Coupled impact remains a separate gate.
