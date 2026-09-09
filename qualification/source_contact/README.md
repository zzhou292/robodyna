# Pinned source-part contact qualification

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
IDs only within this single-member fixture namespace. The source-only extension
also copies authenticated E2a order-16 parent mass values and exact density,
thickness and aggregate metadata; it does not introduce a modelio parser or
admit a mechanics mass, owner, source formulation or attachment.

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

## Prescribed source-force gate (passed)

The first execution passes all four force functions and all four existing
geometry functions on 2026-09-09. Retained evidence is
`crash-work/reports/source-contact-c5c3-tests-1.json` and the matching
`source-contact-c5c3-xml-1/` directory. Maximum measured force/energy errors
are 4.0958957745699387e-7 N and 1.1041728525551851e-12 J. Maximum Q4 cell/visit
counts are 1116/2231. Every one of the 94 positive-contact experiments has a
positive certified resultant lower bound; all three wall tessellations agree
exactly. Budgets below were unchanged. The guard used one CPU affinity slot,
sampled 34,861,056 peak RSS bytes and at least 102.44 GiB available RAM.

`source_part_force_check.cpp` adds four host functions and consumes the same
two authenticated command-line inputs. It uses the owning TL mixed-family
dispatcher, named constant-center-area Q4 model and genuine native T3 integral.
All selected inputs retain the 117-node borrowed space, original source IDs and
native connectivity. One 442,368-byte Q4 scratch is reused; the total fixed
test storage is checked below 1 MiB, excluding parser/wall allocations.

The explicit test mass policy is `qualification-e2a16-equal-native-node-lump`:
divide each authenticated proxy parent mass by its native arity and assemble
its shares once into the common physical-node map. Pass that actual chosen
mass and fixed mask into the owning Jacobian. This differs from contact A0 and
is **not** an admitted source mass/lumping or rotary-inertia policy. Independent
raw-token checks preserve all loaded mass/density bits.

Each original parent first runs separated, then undergoes its own rigid X
translation so its deepest represented node approaches 0.25 mm penetration.
Y/Z and all immutable reference coordinates stay unchanged. These 94 separate
experiments preserve each source shape; their different translations do not
constitute one whole-part motion. Another test applies a single rigid shift to
all 117 nodes, assembles all 94 contributions, checks reaction/moment/power and
stages the whole test result across a late invalid-mass failure and clean retry.
Original/diagonal-flip/subdivided actual finite walls must give identical
same-state forces and certificates for every parent.

The qualification parameters are fixed before execution: kappa 4e5 N/m³,
maximum depth 0.5 mm, force error 5e-7 N and potential error
1.2500000000000005e-12 J. Q4 limits stay 4096 leaves, 16384 visits and depth16
per axis; T3 uses at most two nominal subtriangles/six samples. Independent
long-double source areas and positive-corner pressure bounds supplement the
owning analytic unit gates. A positive source force may remain unresolved from
zero within the absolute certificate; its count is reported explicitly rather
than called a relative-accuracy pass. This does not compare the declared Q4
model with the deferred variable-midsurface-density alternative.

Root owns registration, bounded execution and retained evidence. The registered
force target is `robo_dyna_source_part_force_check`, with the same fixture
arguments shown above and an additional link to the owning prescribed-surface
contact library. No dynamics, native shell-card equivalence or vehicle result
follows from this prescribed-force gate.
