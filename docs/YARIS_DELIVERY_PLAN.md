# Robo-dyna: Yaris delivery and product roadmap

Updated 2026-09-24 15:20:13 UTC. The recovered two-consecutive-interval wall+self checkpoint,
closed archive, exact readback and native Chrono replay are complete at 400 ns.
The historical composite GTest failure remains recorded; recovery did not rerun
physics. [Evidence](../../crash-work/reports/gate14-readback-recovery.json) establishes short correctness, not a useful
crash video or long-duration validity. Heavy execution has been resumed; keep
other GPU work running under the active guards.

The [current execution plan](../../planning/CURRENT_EXECUTION_PLAN.md) owns source pins/resources and
[architecture guide](ARCHITECTURE.md) owns the source map.
The historical first-interval M2 evidence and this [preserved previous version](../../planning/history/2026-09-24-before-gpu-certificate-priority-refresh/robo-dyna/docs/YARIS_DELIVERY_PLAN.md) remain preserved.

## Implemented result and next milestones

The selected coarse 2010 Yaris V5 assembly runs CUDA explicit structural dynamics
with QEPH/T3/QBAT shells and plasticity, solids, beams, joints, connections, rigid
groups and CIN. TL remains the single physical owner; Chrono replays exact accepted
shell states with original part colors. The contact pipeline remains hybrid;
GPU native verification is active development, not an already GPU-complete solver.

The longest visible archive remains **wall-only**: 10,000 steps/~2 ms, 41 saved states,
about 815 MB, peak local shell equivalent plastic strain 1.94975%. This does not
prove settled crush, rail load transfer, fracture or a 5 ms wall+self trajectory.
Reviewed [impact detail](../../crash-work/renders/yaris-wallremoval-10000-review-1/impact-detail-video/movie.mp4)
and [overview](../../crash-work/renders/yaris-wallremoval-10000-review-1/overview-video/movie.mp4)
remain available; no new useful self-contact movie exists yet.

| Milestone | Current state | Completion gate |
|---|---|---|
| Two actual wall+self commits and replay | Recovered gate14 COMPLETE;400 ns | Preserve original failed receipt plus successful same-archive recovery |
| CPU performance staging | 17dc includes common-point/narrow work and sorted lookup; coupled 171 host + 34 CUDA checks pass | Owning integrated host/CUDA/fixture/rollback gates |
| General GPU native verification | Shared primitives and five Bazel targets pass; instance policy source underway; no native kernel/batch integrated | Exact CPU/GPU full results and fallback, source/phase, resource and publication qualification |
| Standalone GPU facet filtering |3 host/6 CUDA tests and 16.15x component result; not integrated | Bounded transaction composition with exact parity and measured vehicle gain |
| Canonical CPU witness search | DEFERRED despite110 tests/stack pass; benchmarks inconclusive | Do not select without reliable benefit; preserve evidence |
| Strict delivery wall law | Explicit1e10 N/m³ /.002 m, pending combined qualification | Fresh forecast and nonzero response under unchanged stability/contact screens |
| Useful 5 ms self-contact impact | NOT STARTED; current~600 s/step is impractical | Practical measured rate,25,001 accepted intervals, closed authenticated output |
| Reviewed video | Existing wall-only video; new self-contact video incomplete | Exact Chrono replay, original part colors, scale1, true timestamps, visible deformation, full decode |
| Later diagnostics/restart/contact profiles | Incomplete | Independent energy/load/failure ledgers, true restart parity and named new physical profiles |

The ordinary delivery configuration uses20 mm gap,51 samples and the explicit
strict wall law. The gate14 speed-control defaults remain 1e7/0.1 and 1 µm gap.
Do not mix their claims. The [existing CLI/readiness plan](../../planning/YARIS_WALL_SELF_LONG_RUN_READINESS_2026-09-24.md)
and [separate wall-law proposal](../../crash-work/reports/yaris-wall-self-delivery-wall-law.proposed.json)
require fresh combined qualification and forecast before launch. No new controller
or rendering engine is required, and no physical restart is available.

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

GPU exact verification is the critical path: gate14 spent about 350 s/step in native
certificates. Standalone filtering addresses less than 75 s and cannot by itself
make the approximately 600 s step practical. Component ratios are not multiplied
into a vehicle ETA. Do not launch 5 ms until measured consecutive steps justify it.
Preserve one TL owner/clock, exact candidate inventory, original material/timestep,
error order and rollback. Reuse coherent private math/geometry/batch modules and
owning tests; avoid broad unqualified rewrites or vehicle-ID shortcuts.

The [active resource rules](../../planning/CURRENT_EXECUTION_PLAN.md) retain 4 CPUs/10 GiB RSS for vehicle
qualification, shared GPU0 with 8 GiB free reserve and 6 GiB growth, 32 GiB host
reserve, normal 20 GB app forecast and 2 GiB archive/per-view capture caps. Heavy
jobs are serialized through the shared guard/lock; other GPU work continues.
No remote pushes. Conditional larger allowances are not an automatic cap increase.

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
