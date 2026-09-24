# Robo-dyna: Yaris delivery and product roadmap

Updated 2026-09-24 20:27 UTC.

**The optional native-GPU vehicle path is unqualified and unselected.** The
combined TL `233558c3` / app `a6eb920d` probe reached its 1,800 s guard timeout
without an observed completed interval. Guard elapsed was 1,800.344 s, exit 125,
with complete cleanup. Only the initial epoch-0 frame exists; the archive is
incomplete, with no summary/viewer input, frame comparison or Chrono replay.

The selected historical CPU vehicle baseline remains TL `17dc769e` / app
`667bdd5`: one authenticated 200 ns prefix and exact Chrono replay, with
270.592557 s Prepare. The longest combined wall+self checkpoint remains recovered
gate 14 at 400 ns. The reviewed 10,000-step/~2 ms videos remain **wall-only**.
Useful long-duration self-contact footage and physical restart are unfinished.

## Latest result and next milestones

Combined TL owning qualification passed 253 host and 57 CUDA/determinism GTests,
314 complete adaptive/wide discovery records, O0 forecast linkage, source/syntax/
shape and seven Bazel targets. The app then passed 121 host and two GPU GTests,
with a GTest-free CLI and byte-identical qualified native object. These results
qualify interfaces and tested numerical behavior, not real-vehicle throughput.

The full GPU-enabled vehicle timeout is preserved in
[its diagnostic](../../crash-work/reports/wall-self-cli-combined-one-interval-1.timeout-diagnostic.json)
and `wall-self-native-gpu-timeout-20260924-1`. Peak sampled RSS stayed below 10 GiB;
the stop was the 30 min time guard. GPU time-busy is not occupancy evidence. No
correct-frame comparison or Chrono replay exists for this incomplete attempt.

1. Finish the **same-binary CPU-native control**, root-owned, using unchanged
   TL `233558c3` / app `a6eb920d`, compact CUDA facet queries still enabled and the
   native CUDA option off. Check the exact policy/state payload and actual Chrono
   replay before claiming a new vehicle-qualified producer or performance gain.
2. Use its coarse discovery/filter/policy timings to choose the next work. Output
   feature ordinal sorting is a source-reviewed design only; do not implement it
   unless measured output sorting matters. The earlier input-ledger experiment
   stays unselected after mixed component results.
3. Diagnose native CUDA on **representative bounded workloads** before another
   vehicle GPU attempt. The earlier fast eligible benchmark had 3,072 pairs and
   exactly 3,072 visits; it did not exercise multi-visit GPU jobs. Measure cohort
   work distributions/maxima, kernel/transfer time and actual difficult-pair
   replays. Canonical witness ordering may reduce contact predicates but cannot
   shorten work-exhausted DFS tails. Zero-neutral storage is isolated and still
   needs full qualification/measurement; it is not a fix proved by this timeout.
4. Establish practical consecutive-step throughput with unchanged physics and
   complete candidate/policy/rollback semantics. Then qualify the stricter wall
   law, forecast the existing 5 ms/51-sample route, and render actual accepted
   states with original part colors, scale 1, real timestamps and visible crush.

The selected assembly already has CUDA explicit structural dynamics, shell
plasticity, solids/beams, rigid/CIN/connection support and first-profile self-contact.
Its practical full-vehicle runtime remains the delivery blocker. Use existing
CLI/controller/archive and Chrono components; no new rendering engine is needed.

Reviewed **wall-only** [impact detail](../../crash-work/renders/yaris-wallremoval-10000-review-1/impact-detail-video/movie.mp4)
and [overview](../../crash-work/renders/yaris-wallremoval-10000-review-1/overview-video/movie.mp4)
remain available. They are not longer self-contact evidence.

The [execution plan](../../planning/CURRENT_EXECUTION_PLAN.md) and
[GPU plan](../../planning/GPU_THROUGHPUT_PLAN.md) own sequencing, guards and current
promotion decisions. Keep other GPU work running. The speed control stays1e7/.1
with1µm gap; delivery proposes1e10/.002 and20 mm gap after separate strict qualification.

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

The historical selected CPU one-interval profile has several expensive stages: filtering 76.397 s,
discovery 56.696 s, candidate policy 51.271 s and native verification 47.948 s.
A useful long run is still impractical; component gains are not multiplied into
an ETA. Keep the CPU reference path until measured replacements pass their gates.
The composed native GPU facade already preserves one initialized fallback pool;
its vehicle runtime remains unqualified after the timeout.

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
