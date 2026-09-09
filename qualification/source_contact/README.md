# Pinned source-part contact geometry check

All four source-backed checks pass (2026-09-09); all 94 original parents have
qualified intrinsic reference geometry and covered prescribed sweep boxes.
The report is `crash-work/reports/source-contact-tests-1.xml`. This optional
host qualification input is the exact
E2a `yaris-part-2000157-readiness-1.json` report: 671,971 bytes, SHA256
`74e733b76a5c530c94ba39876ce3707df2ca8c602348204632eb902a48402d89`.
Its existing producer uses `modelio/canonical_geometry.py` to check the original
archive, selected raw source cards, canonical array hashes and complete shell
coverage. The [E2a checkpoint](../../../crash-work/reports/e2a-readiness-checkpoint-1.json)
retains that verification. This consumer verifies once-read report bytes before
parsing its geometry with the existing bundled full-precision RapidJSON; it is
not a second deck parser or a general readiness-file loader.

`SourcePartContactFixture` preserves all 117 source nodes and 94 source shell
records, their IDs, raw four-node slots, canonical/local indices, source lines,
blank masks and coordinate bits. Native T3s expose three physical nodes while
retaining the repeated fourth raw slot. Stable feature IDs equal source element
IDs only within this single-member fixture namespace. No mass, owner, force,
time integration, source formulation or attachment is admitted here.

The four focused tests check independent raw-token coordinate conversion and
every source mapping; all 88 Q4 and six native T3 intrinsic reference measures;
long-double point-density values against the directed bounds; and complete
finite-wall coverage of explicitly prescribed endpoint boxes. The actual pinned
wall is loaded through `CanonicalWallArtifacts`, `CanonicalWall` and original
`WallTessellation`, preserving its 62 vertices/100 triangles. The three original
mixed-sign projected Q4s must be included. Bad reads/hashes, wrong typed requests
and retry check staged publication. No test skips on missing required assets.

The whole-area bounds and their ordinary diagnostic sums are coarse only;
they cannot replace variable Q4 density in contact. The existing E2a mass
interpretation is not promoted to a source solver mass policy. These tests
also do not qualify force integration, moving-source dynamics or a vehicle.
Analytic exactly-edge-on and exposed-boundary cases belong to the owning TL
C5c.1 tests; this app fixture uses the original unmodified source geometry.

CMake option `ROBO_DYNA_ENABLE_SOURCE_CONTACT_CHECKS=ON` enables the check.
Set `ROBO_DYNA_SOURCE_PART_READINESS` and `CRASH_CANONICAL_WALL` to the existing
authenticated artifacts, and select the existing Chrono prefix. The test
executable accepts both fixtures, followed by GTest options:

```text
robo_dyna_source_part_contact_check \
  crash-work/reports/yaris-part-2000157-readiness-1.json \
  crash-work/assets/yaris-wall/manifest.json
```

Compile the fixture adapter and check with C++17, the owning TL material-measure
and planar-wall geometry targets, robo-dyna `artifact_io`, canonical wall and
wall-tessellation helpers, and GTest (this source provides `main`). Use strict
binary64/no-fast-math/no-FTZ flags, matching the owning TL bounds contract.
The fixed fixture storage is statically below 32 KiB; JSON parsing and the
finite-wall owner are bounded host-only allocations. Parents are evaluated one
at a time; the 64-node dynamics owner capacity is untouched.
