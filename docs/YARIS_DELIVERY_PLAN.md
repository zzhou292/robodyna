# Robo-dyna: Yaris delivery and product roadmap

Updated 2026-09-24 17:09 UTC. The selected TL `17dc769e` / app `667bdd5` path passed a real
200 ns public-CLI prefix and exact Chrono replay at 270.592557 s Prepare time,
2.18146x faster than the matching gate 14 interval. The longest combined wall+self
archive remains recovered gate 14 at 400 ns. These short checkpoints do not yet
provide a useful self-contact crash movie or a sustained stepping rate.

The [execution plan](../../planning/CURRENT_EXECUTION_PLAN.md) owns sequencing and
resources; the [GPU throughput plan](../../planning/GPU_THROUGHPUT_PLAN.md) owns
measurements and promotion. Keep other GPU work running under the existing guards.

## Implemented result and next milestones

The selected coarse 2010 Yaris V5 assembly runs CUDA explicit structural dynamics
with QEPH/T3/QBAT shells and plasticity, solids, beams, joints, rigid groups and CIN.
TL remains the single physical owner; Chrono replays accepted shell states with
original part colors. Contact execution is still hybrid.

The longest visible archive is **wall-only**: 10,000 steps/~2 ms, 41 saved states,
about 815 MB, peak local shell equivalent plastic strain 1.94975%.
Reviewed [impact detail](../../crash-work/renders/yaris-wallremoval-10000-review-1/impact-detail-video/movie.mp4)
and [overview](../../crash-work/renders/yaris-wallremoval-10000-review-1/overview-video/movie.mp4)
remain available. They do not prove a longer self-contact impact, settled crush,
rail load transfer or fracture.

1. Qualify the optional CUDA facet-filter transaction adapter. Its source/syntax
   review passes; owning runtime, ordering, rollback and resource gates are next.
2. Measure the qualified discovery index-ledger change. Its 58 tests and 314-record
   parity pass; its reviewed comparison benchmark has not yet established a gain.
3. Advance bounded CUDA native numerical cohorts. Raw/compound/worker-capacity
   parity is qualified, but mixed measurements showed at most 1.0135x, so it remains
   unselected. Eligible-only diagnostic subsets show a useful larger-cohort effect;
   every omitted pair/work amount is explicit and no production contact is removed.
   A separate cache implementation needs exact per-slice semantics and measured
   transfer-inclusive benefit before selection.
4. Compose only measured winners and run a bounded combined vehicle probe.
   Then qualify the stricter wall law and practical consecutive-step throughput.
5. Run the existing 5 ms/51-sample delivery path and render actual states with
   original part colors, scale 1, true timestamps and visible deformation. Require
   a closed authenticated archive, exact Chrono replay and full video decode.

The ordinary delivery proposal uses 20 mm gap and explicit 1e10 N/m³ stiffness /
.002 m penetration limit. The regression control remains 1e7/.1 with 1 µm gap.
The stricter combined law is not qualified yet. Follow the existing
[readiness plan](../../planning/YARIS_WALL_SELF_LONG_RUN_READINESS_2026-09-24.md) and
[wall-law proposal](../../crash-work/reports/yaris-wall-self-delivery-wall-law.proposed.json)
with a fresh full forecast; no new controller or rendering engine is needed.
There is no physical restart capability.

## First-profile contact scope

Current self-contact is frictionless, centered selected Q4/T3 shells, reference
half-thickness and level-0 fixed physical triangles. Virtual facet vertices use
original interpolation weights and add no mechanical nodes or mass. Original
friction/damping/soft-card fields remain provenance, not applied physics.

Implemented: canonical feature/active-parent identity, complete bounded CUDA
sweep, VF/EE discovery, local intersection ownership, same-rigid/CIN support,
reference-area force, represented-Jacobian STI, exact linear/rigid-quadratic
interval certificates, deterministic force reduction and common commit/discard.
It is not exact bilinear-Q4 contact, and selected solids/beams are not surface
primitives. Friction/history and broader contact semantics remain future scope.

Combined first-interval evidence records 32,491 active events and 5,989,588
candidate facet-policy outcomes, digest 5411954021770061371. The standalone
self-only guard pass is verified; identical self-only printed counts remain
handover-reported because its final log was not recovered. The audit preserves
all provenance limitations and the exact partition.

The existing wall remains a fixed-X finite planar authenticated mesh penalty;
self-contact adds panel-to-panel handling for its declared profile. Do not infer
arbitrary moving-wall or full original LS-DYNA contact parity.


## Performance and architecture constraints

The latest one-interval profile has several expensive stages: filtering 76.397 s,
discovery 56.696 s, candidate policy 51.271 s and native verification 47.948 s.
A useful long run is still impractical; component gains are not multiplied into
an ETA. Keep the CPU reference path until measured replacements pass their gates.
The native GPU facade must not create a second initialized CPU fallback pool when
composed into transactions.

Preserve one TL owner/clock, complete candidate inventory, material/timestep,
error order and rollback. Reuse modular math/geometry/batch utilities and owning
tests. No vehicle-ID shortcuts, dropped production pairs or silent CPU retry after
CUDA failure. Visualization archives are not restart state. The current first
profile does not imply full LS-DYNA equivalence.

The [active resource rules](../../planning/CURRENT_EXECUTION_PLAN.md) retain 4 affinity
CPUs/10 GiB RSS for vehicle qualification, shared GPU0 with 8 GiB free reserve and
6 GiB growth, 32 GiB host reserve, normal 20 GB app forecast and 2 GiB archive/per-view
capture caps. Serialize heavy jobs through the shared guard; no remote pushes.
Conditional larger allowances require an explicit forecast.

## Selected model and failure boundary

V5 includes 867 shell parts / 349,645 shells, 376,930 physical nodes, 4,980 solids,
142 structural beams, 4,442 TYPE13 and 2,828 TYPE25 connections, 154 mass cards,
44 joints, 779 rigid groups and 11,165 CIN rows. Selected mass is
704.35196065175535 kg, not full vehicle curb mass. Its one static potential
load-transfer component is not a DOF-rank or loaded-transfer proof.

The original 861-PID contact selection resolves to 337,092 centered shell
parents in 842 retained PIDs and 653,055 level-0 facets. The eight tire PIDs /
8,812 shells remain omitted. It also selects 2,952 solids in 11 PIDs, whose
contact faces are unsupported here; selected beam contact is absent.

Other source omissions and explicitly selected rubber HEPH/S6Z and airbag
Isolid18 execution choices remain documented in modelio. Do not erase original
Isolid1/Isolid5 provenance or claim original trajectory equivalence. Four seat
rigid groups retain restricted policies. LAW44 EPMAX does not authorize generic
solid erosion. TYPE25 huge default rupture limits are not calibrated tearing.
Removed-shell degeneracy remains an admission boundary during severe collapse.

Preserve the original 2 ms result and failed receipts. Visualization archives and
folders of frozen binaries are not physical restart checkpoints.

The previous delivery plan is preserved in
[entry-document history](../../planning/history/2026-09-24-before-gpu-native-benchmark-checkpoint/robo-dyna/docs/YARIS_DELIVERY_PLAN.md).
