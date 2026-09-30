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

## Prescribed CUDA source gate (passed)

`SourceContactForceFixture.h/.cpp` share the host fixture and mass/path helpers
with `source_part_contact_cuda_check.cpp`. All four host functions and all 392
saved diagnostic values remain unchanged after this extraction.
`SourceContactCudaFixture.h/.cu` contain only the bounded qualification packet
and two-worker launch of the existing TL primitives. Three actual GPU functions
pass their first numerical execution in
`crash-work/reports/source-contact-c5c4-cuda-tests-1.json/xml`.

The two one-thread worker blocks reuse separate Q4 scratch regions. One startup
allocation owns 1,522,464 device bytes; there is no allocation per parent. The
test covers all 94 source parents, separated and independently translated into
contact, original/flipped/subdivided finite-wall host preflight, one coherent
whole-part profile, failed late parent preservation and exact fieldwise clean
retry. CPU/GPU truth enclosures overlap at the unchanged budgets, with active
area checks and all actual source identities. Returned contributions are reduced
on the host; this does not qualify connected GPU force assembly or a transaction
across parents. Native T3 exports its three translation forces, while Q4 direct
couples are checked to be zero. The host fixture still owns finite mesh coverage.

Maximum measured kernel event time is 4,360.626953 ms for the independent-parent
profile; the coherent whole-part profile takes 444.561310 ms. These measurements
make this small sequential-worker launch unsuitable as a production timestep.
Keep it as reference evidence while a production execution/discretization choice
is evaluated. No vehicle throughput claim follows from CUDA parity.

The first CUDA compilation exceeded its original 6 GiB RSS guard. Keeping the
large scalar integrators out of worker-loop expansion (`__noinline__` wrappers,
no loop unrolling) reduced the next compile's sampled peak to 393,609,216 bytes.
An unrelated test compile error assumed a T3 couple field that its force-only
type does not contain; the test now respects that native interface. No numerical
law or error tolerance changed. The successful run took 27.909 s under the
external guard, with 180,219,904 bytes sampled peak RSS and at least 100.34 GiB
available RAM. GPU context memory is additional to the explicit allocation.

Enable `ROBO_DYNA_SOURCE_CONTACT_CUDA=ON` alongside the existing source-contact
option and use target `robo_dyna_source_part_contact_cuda_check`. Missing CUDA
is a failing required gate, never a skipped success. Keep FP64 contraction,
fast math and flush-to-zero disabled, and serialize the run with the workstation
guard. New heavy builds may use 12 GiB RSS under the user's 16 GB RAM ceiling;
the numerical test retains its smaller 2 GiB cap.

## Native source-shell startup and Q4 rates

The fixture also supports two explicitly enabled structural qualification
operations. `ROBO_DYNA_SOURCE_T3_STARTUP` passes two functions on all six
original triangles, using the isolated native starter frame and selected
angle-weighted mass/inertia expressions. Its seven standalone native tests
are retained in the owning TL reference package.

The separate `ROBO_DYNA_SOURCE_T3_RATES` gate now passes two functions on all
six original triangles, alongside the two unchanged startup regressions.
`source_part_t3_rates_check.cpp` consumes the same authenticated readiness
input and shared test-only source mapping. The complete native C3COOR3,
C3EVEC3, C3DERI3, C3DEFO3 and C3CURV3 leaves produce the current geometry and
eight rates from prescribed endpoint positions and midpoint velocities.
Original coordinates and IDs remain unchanged; the independent long-double
affine/angular oracle uses the frozen R2 dimensional budgets. Evidence is
`crash-work/reports/t3-r2-source-{build,tests}-1.json` and
`t3-r2-source-xml-1/`. The executable is
`robo_dyna_source_part_t3_rates_check READINESS.json`. This qualifies prescribed
rates only; material history, forces, a temporal owner and source MAT024/NIP3
admission remain outside this T3 gate.

`source_part_t3_force_check.cpp` passes two separate native R3 functions:
48 material/force modes across the six original triangles, then
load/hold/reversal, failed-sample retry and complete C3UPDT3 contributions mapped
once to the original 117-node host space. It reuses the authenticated input and
independent T3 rate/material/virtual-power oracles. Root registers the optional
target with `t3_r3_native`; there is no Fortran production dependency or dynamics
owner. The native branch is explicitly LAW1/NPT0/ISH3N2, with original geometry,
density and thickness; original MAT024/NIP3 behavior remains unconsumed. The
[source report](../../../crash-work/reports/t3-r3-source-tests-1.json) and
[XML](../../../crash-work/reports/t3-r3-source-xml-1/) retain both new functions
and all four startup/rates regressions. Together with the standalone native
gate, all 28 executions pass; these are force/history checks, not CUDA T3
mechanics or connected dynamics.

`ROBO_DYNA_SOURCE_QEPH_GEOMETRY` passes three host functions on all 88 original
quads. Startup matches the qualified native QEPH reference, with independent
long-double diagonal-cross area and separate physical/area-added inertia
checks. Contributions assemble once in the original 117-node space; nodes
used only by T3 remain zero in this Q4-only sum. The Q4 subtotal is
0.24790758675157906 kg and the chosen isotropic inertia subtotal is
4.9789918392944437e-06 kg m². These are chosen QEPH startup quantities, not a
full-part structural mass admission or source ELFORM2 equivalence.

All 264 current-geometry/rate configurations (three existing prescribed patterns
per quad) match all 80 native fields at the unchanged Q3a/Q3b dimensional
2e-12 budgets. Source coordinates, density, thickness and IDs are unchanged;
E=200 GPa and nu=.3 are explicitly supplied experiment metadata. MAT024 history,
forces, source CUDA batches and dynamics are not established by this gate.
The first build/run passes without repair or threshold changes; reports are
`crash-work/reports/source-qeph-geometry-{configure,build,tests}-1.json` and
`source-qeph-geometry-xml-1/`. The executable is
`robo_dyna_source_part_qeph_geometry_check READINESS.json`.

Both options reuse this exact authenticated fixture and enable the local GNU
Fortran oracle only in qualification targets. Neither links Fortran into the
production solver. Keep native T3 and QEPH context/module symbols private and
retain their independent startup formulas and original native node counts.
