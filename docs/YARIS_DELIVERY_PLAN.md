# Robo-dyna: Yaris delivery and product roadmap

Updated 2026-09-17 after the accepted M2 source/evidence audit and local app
consolidation. See [the active plan](../../planning/CURRENT_EXECUTION_PLAN.md),
[acceptance audit](../../planning/handover/M2_ACCEPTANCE_AUDIT_2026-09-17.md), and
[post-M2 design](../../planning/handover/POST_M2_EXECUTION_2026-09-17.md).

## Delivered and accepted

The selected coarse 2010 Yaris V5 assembly already runs CUDA explicit structural
dynamics with QEPH/T3/QBAT shells and plasticity, five solid families, beams,
joints, connections, rigid groups and CIN. Chrono replays exact accepted shell
states with original part colors. TL remains the physical state/clock owner.

The longest archived run is still **wall-only**: 10,000 steps / about 2 ms at
200 ns, 41 saved states, 102.44 minutes and approximately 815 MB. Peak local
shell equivalent plastic strain is 1.94975%; 76 positive native points belong
to 49 active parents of bumper PID 2000003. M0 analysis is complete. It does not
prove rail load transfer, fracture, settled residual crush or the configured
5 ms horizon's completion.

The inspected existing videos remain available at
[impact detail](../../crash-work/renders/yaris-wallremoval-10000-review-1/impact-detail-video/movie.mp4)
and [overview](../../crash-work/renders/yaris-wallremoval-10000-review-1/overview-video/movie.mp4).
Each is 8.2 s, 1280x720/30 FPS, physical scale 1, with saved-time overlays and
no geometric interpolation. M2 has not produced a longer movie.

**Functional M2 first-profile self-contact is accepted.** Full selected V5
self-only prepares/seals/discards; combined wall+self seals both receipts and
commits one 200 ns interval to epoch 1. Both final resource receipts pass with
complete cleanup. This completes the named M2 integration milestone, not the
whole crash deliverable or multi-step production controller integration.

Accepted TL source is `1cf575e6` in
`crash-work/worktrees/self-contact-m2-final-integration`. App M2 source was
`9606129`; local merge `de074ed` retains that runtime and newer M0 analysis.
Root app and runtime worktree share the merged history, followed by docs-only
updates. Accepted simulation sources did not change during consolidation.
TL root remains the older `1e6a0a2`; configure accepted TL explicitly.

## Remaining capabilities and gates

| Priority / slice | Reuse | Remaining implementation | Completion gate |
| --- | --- | --- | --- |
| P0: runnable wall+self controller | `LoadedWallSelfContact`, `PreparedRun`, existing source/setup factories and one dynamics owner | Explicit contact mode through source preparation, config/CLI and Session; current controller still calls `LoadedWall` | Small selection/identity tests, no hidden mode change, correct common publication across multiple commits |
| P0: bounded multi-step budgets/output | Existing combined forecasts, `SelfContactObservation`, typed transaction reports and archive writers | Shared-source/phase-peak accounting, declared growing-contact capacities, compact accepted contact summaries, profile provenance and typed failure propagation | Exact caps and source identity, no duplicate memory charges, failed trials never become accepted output, bounded failure leaves valid diagnostic prefix |
| P0: next-interval qualification | Existing TL two-step force/STI, discard/retry and removal CUDA coupons | App/controller wall+self fixture with several actual commits and archive capture, then bounded full V5 continuation beyond epoch 1 | No stale receipts, correct epochs/history, stable allocation and supported motion; final full vehicle stage/resource evidence |
| P0: practical contact throughput | Accepted exact geometry and frozen source-authenticated fixtures | Measure host geometry/certification, state copies, broadphase and force phases; optimize generic scheduling/representation without changing admitted outcomes | Exact policy/force/failure regression; measured representative benefit under unchanged 200 ns and resource limits |
| P0: energy/load-path evidence | Native work/plastic/hourglass terms, accepted force observations, M0 mapping | TIME0/current energy and momentum, contact impulse/work, connector/body reactions and loaded transfer | Correct phase/availability, no double-counted plastic/HG or rigid/CIN mass; free-flight, elastic, unloading/contact controls and stated tolerances |
| P1: integrate separate optimizations | CIN `3bb0cb1`/`28d1f84`; solid `2c40dbb`/`b17fde2` | CIN into accepted contact branch; finish solid integrated/native/V5/exact/performance gates | One change at a time, owner/rollback/caps coverage and unchanged output where applicable; no claim of existing M2 speedup |
| P1: damage/load history | Existing constant/TAB1 shell and TYPE13/TYPE25 failure mechanics, activity/removal | Accepted event export and loaded failure/load-path review | Correct failure/removal timing, changed ownership and no stale force/history; retain source limits |
| P1: physical restart | Authenticated models, bounded I/O and typed accepted owner/participant state | Versioned checkpoint of staggered phase, coordinates/velocities/orientation, all histories/activity/constraints and new contact state | Uninterrupted/resumed next-step equivalence, compatibility/corruption/partial-write rejection and bounded storage |
| P1: 5 ms and longer video | Existing CLI loop, exact archive, Chrono capture and video encoder | Complete a practical wall+self trajectory after preceding gates; admit later horizons explicitly | Reviewed scale-1 part colors, actual time, visible response, contact/energy/load evidence and stable resources |
| P2: broader contact/model/results | Current centered-shell first profile and source provenance | Solid contact faces, exact bilinear Q4, friction/history, omitted source components and richer stress/force plots | Separately named physical profiles and independent owning qualification; no hidden expansion of first-profile claims |

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

The contact pipeline is hybrid: CUDA structural dynamics/broadphase/force and
host exact feature/crossing certification. The roughly 32-minute first-interval
gates include startup and are not measured steady-state throughput. Do not use
the old wall-only 1.63 steps/s rate for a wall+self ETA. The next representative
continuation must report per-stage time and resource use before a long run.

The harness's exact 32,491 event allocation is not a growth policy. Combined
sampled whole-device GPU growth left only about 266 MB below the 6 GiB guard.
Derive generic capacities and phase peaks before expanding storage. Keep normal
20 GB application RAM / 2 GiB archive; conditional 60 GB / 6 GiB requires a
complete forecast. All heavy work uses the shared guard/lock; no pushes.

Preserve the single physical owner and Chrono-style modular composition. Several
transaction/certificate files are now 1400–3200 lines; extract focused helpers
as relevant work touches them, retaining exact operation order and fixtures.
Avoid sweeping rewrites or making cosmetic cleanup block the runnable deliverable.

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
