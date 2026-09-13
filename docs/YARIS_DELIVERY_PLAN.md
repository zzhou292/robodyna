# Robo-dyna: remaining Yaris work and product roadmap

Updated 2026-09-12 from main sources, frozen drafts, the completed 10,000-step
run, and independent contact/runtime/model reviews. The execution order lives in
[CURRENT_EXECUTION_PLAN.md](../../planning/CURRENT_EXECUTION_PLAN.md); persistent
project facts live in [WORKSPACE_MEMORY.md](../../planning/WORKSPACE_MEMORY.md).

## What is already delivered

The original coarse 2010 Yaris **selected V5 assembly** hits a finite triangle
mesh wall at 35 mph using TL-FEA CUDA explicit dynamics, with one accepted/trial
state and clock. QEPH/T3/QBAT shells, layered plasticity, five solid families,
structural beams, joints, original connections, rigid groups and CIN ties are
integrated. Chrono replays exact saved shell states with original part colors.

The completed run reached **10,000 steps / 2 ms**, at **200 ns**, in 102.44 minutes.
Its 41 saved states occupy an approximately 815 MB run directory; peak sampled
RSS was 3.736 GB. First saved positive shell plastic strain is at 0.55 ms; the
final maximum is **1.94975% at 76 positive native points**. All 349,645 shell
parents remain active; reported solid/structural-beam plastic work is zero.
This demonstrates local shell plastic response, not fracture or a settled
permanent shape. The configured 5 ms envelope was intentionally stopped at 2 ms.

Both 8.2-second, 1280x720/30 FPS videos pass exact replay, hashes, probe and full
decoding. They use physical deformation scale 1 and actual timestamp overlays:
[impact detail](../../crash-work/renders/yaris-wallremoval-10000-review-1/impact-detail-video/movie.mp4)
and [overview](../../crash-work/renders/yaris-wallremoval-10000-review-1/overview-video/movie.mp4).

The authenticated saved-response analyzer maps all 76 positive native points to
49 active parents of PID/MID/SID 2000003 (`2_bumperplastic`), with the peak at
EID 2324558. Their direct static incidence remains inside that bumper shell part;
it does not establish reaction transfer into rails. The complete bounded report
is `crash-work/reports/yaris-wallremoval-10000-impact-analysis-3.json`.

## Remaining capabilities

| Priority / piece | Existing work to reuse | Work still required and owning modules | Completion gate |
| --- | --- | --- | --- |
| P0: interpret the current response | The focused authenticated analyzer now reports sampled parent/part/material/section plasticity and direct static incidence from Replay/Context and the V5 graph. | TL/app diagnostics must add online connector/body reactions, scheme-correct energy/momentum and loaded force/moment paths; saved positions cannot establish residual shape. | M0 reproduces all 2 ms extrema and source joins. Remaining gate is measured transfer beyond the bumper skin in controlled/full-front loading; one weak graph component remains insufficient. |
| P0: finish a bounded throughput pass | Two isolated CIN master-gather and solid-measurement drafts; existing incidence/gather, exact comparison and profiler. | TL `solvers/cin_advance` and `elements/solids/resident`: finish qualification and integration. Then profile the deformed workload before further optimization. | Frozen CPU/CUDA numerical equivalence including plastic/failure states, rollback/retry, exact cap accounting, affected owning gates and exact V5 outputs. Demonstrate measured whole-step benefit; no physics or timestep changes. |
| P0: vehicle self-contact | Qualified S0/S1 plus locally composed pair/facet/broadphase and bounded approximation summary at TL `79bcf15`; app `34d58e5` qualifies the actual source intersection/facet inventory. Neither branch is merged or runtime-active. | TL `collision` implements feature discovery and a two-sided contributor; app `case/vehicle_dynamics` composes it into the existing transaction. | Complete current regularity/VF/EE/intersection/crossing discovery, equal/opposite force and virtual work, actual rigid/CIN response and STI, same-attempt receipt, no tunneling in covered motion, failure-atomic discard/retry and bounded storage. |
| P0: contact coverage for folding | Existing finite planar mesh-wall law, physical source maps and weighted queries. | Qualify the named fixed physical-facet approximation, vertex-face and edge-edge queries, crossings, thickness/offsets, feature ownership, initial overlaps and local exclusions. Add friction/source-law support in a distinct increment. | Tiny exhaustive geometry oracle; grazing/crossing/degenerate cases; adjacent and tied features; removal changes ownership; wall/contact mesh and timestep refinement; sliding dissipation when friction is enabled. |
| P0: energy and stability evidence | Native force/work/HG/plastic increments, contact/removal diagnostics, fixed-step limiter and source attribution. | TL exposes missing scheme-consistent observations; app `case/vehicle_run` and output assemble a TIME0-to-current energy/momentum ledger and histories. | Free-flight and elastic conservation, plastic unloading, contact work/impulse and removal accounting; count physical/rigid/CIN contributions once. Define tolerances before tests and show timestep refinement. Native plastic/HG work are included components, not additional energy to double-count. |
| P1: damage and connection behavior | Constant/TAB1 shell failure; TYPE13 plastic/failure and TYPE25 force/couple failure; common activity/removal and rollback. | Report accepted connection/failure events through app output. Qualify loaded failure and changed load paths; explicitly resolve any new rupture/erosion policy. | Native failure-step/next-step timing, load transfer after failure, contact ownership after removal, no stale forces/history, source-policy provenance. Current TYPE25 huge default limits are not calibrated tearing. |
| P1: physical checkpoint/restart | Source authentication, bounded binary I/O, typed participant state/history and one publisher. | New versioned checkpoint adapter spanning TL owner/participants and app run/output. Visualization archives and frozen executables are not restart checkpoints. | Uninterrupted versus resumed coordinates, staggered velocity phase, quaternions, all histories/activity/constraints and next accepted results agree; reject incompatible/corrupt files and incomplete writes. Forecast storage before allocation. |
| P1: longer reviewed runs | Standalone run CLI, independent sampling, cooperative prefix closure and automated Chrono capture/encoding. | Complete 5 ms, then an admitted 10/20 ms trajectory; progress to 50/100/200 ms only as contact, output and runtime gates support it. Add explicit horizon admission where absent. | No unexplained energy growth/crossing/penetration failure; inspect deformation and load paths; preserve exact archives, stable memory and a useful part-colored video at each promotion. |
| P2: richer engineering results | Shell geometry/plasticity/activity, part and plastic-color modes, solid/beam scalar summaries and typed accepted-result readers. | Bounded stress/strain, solid/beam geometry and point histories, connector-event/contact-force/energy charts; app output schemas and Chrono adapters. | Known-field export/replay checks, units and unavailable-field semantics, corruption tests, no fabricated fields or second solver in the viewer. |

## Contact design decisions

The current participant is a **fixed-X finite planar mesh-wall penalty**, with
zero thickness offset, friction and damping. The selected wall is an authenticated
mesh; this is not an arbitrary mesh-to-mesh contact engine. It cannot stop car
panels intersecting each other during folding.

Keep the first self-contact scope explicit: frictionless, reference-thickness,
fixed physical facets derived from Q4/T3 parents. Virtual facet vertices retain
original-parent interpolation weights and introduce no new mechanical nodes or
mass. Compose velocity and transpose-force maps consistently; measure/refine
faceting error and retain its conservative search bounds. Reuse existing query
and broadphase utilities. Display triangles do not become physical contact data.
An exact bilinear profile is a later, separately qualified capability.

S0's all-shell census is **341,143 centered parents / 8,502 offset exclusions**.
The authenticated original 861-PID automatic set resolves to 337,092 retained
shell parents in 842 PIDs, all centered, plus 2,952 selected solids in 11
non-shell PIDs and zero beams. The eight selected tire PIDs / 8,812 shells remain
explicitly omitted. Levels 0/1/2 contain 653,055 / 2,612,220 / 10,448,880
facets; maximum reference faceting bounds are 4.42058 / 1.10514 / 0.276286 mm.
Solid exterior faces remain unsupported. Define active-use thickness/area
ownership and narrow topology/tie/rigid exclusions. Same part ID or a shared
corner cannot exclude whole parts.

Pair `0ac4462` (weighted-map dependency `7a629e7`), physical facets `3bd26db`
and broadphase `9bf7ab5` are composed and focused host/CUDA/Bazel-qualified on
the local TL branch. Complete triangle vertex-face/edge-edge queries and bounded
crossing checks next. Swept bounding boxes alone do not establish collision
detection between endpoints; rigid arcs and feature changes need coverage too.
Add force/STI before the existing kick/CIN screen and candidate validation before the common commit.
Persistent friction/history must join that same publication protocol.
See [the contact design](../../planning/DEFORMABLE_SELF_CONTACT_PLAN.md).

## Selected-model coverage and failure limits

V5 includes 867 shell parts / 349,645 shells, 376,930 physical nodes, 4,980 solids,
142 structural beams, 4,442 TYPE13 connections, 2,828 TYPE25 welds, 154 mass cards,
44 joints, 779 rigid groups and 11,165 CIN rows. Its qualified graph has **one
potential load-transfer component**, not seven disconnected components. It does
not prove constrained-DOF rank or loaded transfer after changing activity.

Selected mass is **704.35196065175535 kg**, not full vehicle curb mass. Original
source omissions remain explicit: eight tire parts / 8,812 shells; 10,254 solids
and 101 beam records across 14 non-shell PIDs; the orientation-only mass and
auxiliary/setup dispositions. Original 4,685 beams include 4,442 TYPE13 records,
142 structural beams and 101 omitted records. Tires remain optional for the
requested shell demo. Complete original-deck coverage is a separate milestone.

Material and damage scope is branch-specific. Solid LAW44 EPMAX behavior does
not authorize general solid erosion. The airbag-solid execution choice retains
original Isolid5 metadata while explicitly choosing Isolid18. Four seat-disk
rigid groups retain their restricted policy. TYPE25 default rupture limits are
huge finite values (+/-1e30 N and +/-1e27 N m); do not claim calibrated weld
tearing. Existing TYPE13/TYPE25 event diagnostics should be exported before
adding new failure mechanisms. Removed-shell degeneracy remains a numerical
admission boundary requiring attention during severe collapse.

## Runtime, timestep and output policy

The measured 2 ms run sustained **1.630309 steps/s**. Prepare accounts for about
612.196 ms/step; commit, archive and sampled capture together took 11.724 s out
of 6,133.807 s after startup. Lowering video output quality is not the primary
performance opportunity.

| Physical duration | Approximate steps at 200 ns | Current-rate wall time |
| --- | ---: | ---: |
| 5 ms | 25,000 | 4.26 h |
| 10 ms | 50,000 | 8.52 h |
| 20 ms | 100,000 | 17.04 h |
| 50 ms | 250,000 | 42.60 h |
| 100 ms | 500,000 | 85.19 h |
| 200 ms | 1,000,000 | 170.38 h |

These are linear estimates, excluding startup/rendering and future contact cost;
they are neither delivery ETAs nor evidence that those cases are admitted.
The binary64 endpoint rule can add one step. The present CLI names only
0.5/5/20/50 ms; 10/100/200 ms need configuration/output admission work.

All 10,000 accepted rows admit 200 ns under the post-CIN scalar screen; its
minimum is approximately 229.346729 ns. The separate native solid diagnostic is
approximately 188.513 ns, and TYPE45 automatic stiffness depends on dt squared.
The successful short-run source witness is a rear engine-mount tube, not proof
of an unchanged winner throughout 2 ms. Resolve these scopes and perform
sensitivity checks before any timestep change. No mass scaling, skipped checks,
Yaris special cases or changed material/contact laws count as optimization.

Use normal 20 GB host / 2 GiB archive limits; conditional 60 GB / 6 GiB only when
a full-run forecast requires them. Keep the existing shared workstation guard,
32 GiB available RAM and GPU limits. Runtime sampling and video playback cadence
remain independent of the physical timestep. Add restart before routine multi-day
runs; monitor allocation/soak behavior without claiming the absence of all leaks.

## Broader robo-dyna product, after the selected Yaris demonstration

The longer-term LS-DYNA-like goal also needs a reusable case/deck model beyond
the currently source-pinned Yaris workflow, explicit support matrices for material,
element, section, connection and contact options, and reproducible validation
cases. Tire/road and omitted component mechanics, richer fracture/contact laws,
general loads/boundaries and useful analysis workflows belong here when needed.
Implicit/quasistatic or additional coupled-physics solvers would be separate
algorithms with their own gates; they are not prerequisites for explicit Yaris
impact. A general GUI/preprocessor and packaging can follow stable case APIs.
Formal LS-DYNA/physical-test accuracy comparison remains deferred by the user.

Retain TL-FEA ownership of CUDA mechanics, state and the single clock. Robo-dyna
owns source/model/case/output composition. Chrono owns reused infrastructure and
accepted-state visualization. OpenRadioss remains a pinned reference donor, not
the runtime. Reuse existing utilities and add focused modules with meaningful
tests; avoid a parallel solver hierarchy or one large integration script.
