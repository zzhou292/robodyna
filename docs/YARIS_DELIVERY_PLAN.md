# Robo-dyna: Yaris delivery plan

Updated 2026-09-10 after the four-step milestone, against robo-dyna `4b20345`
and TL-FEA `fb801f1`.
This is the concise delivery roadmap. The workspace
[active execution plan](../../planning/CURRENT_EXECUTION_PLAN.md) tracks current
runs and integration order; [execution status](../../planning/EXECUTION_STATUS.md)
retains the detailed qualification history and failed attempts.

The first vehicle deliverable remains the original **2010 Yaris coarse V1l at
35 mph into a stationary finite triangle-mesh wall through 200 ms**, with accepted
deformation archives and an inspected video. The longer-term product direction
is LS-DYNA-like CAE functionality. The full vehicle is not running.

**Current engineering estimate: about 30% toward that vehicle deliverable.**
This reflects implemented capabilities and remaining model coverage, not test
counts, code volume or the fraction of the requested physical duration. It is
not an estimate of coverage of all LS-DYNA functionality.

## Verified current result

The complete selected **six-part component** runs with **915 shells / 1,030
nodes**, all six internal nodal-rigid groups, six source materials/sections and
two curves. Native QEPH/T3 shell mechanics, source-derived layered plasticity,
rigid-group dynamics, finite mesh-wall contact and accepted output share one
TL-FEA CUDA state and one accepted/trial transaction. External connections are
explicitly released and recorded; this is a detached component, not a closed
vehicle structure.

Its latest **8 m/s** response completes **1.953125 ms** with **104 accepted
frames** and a **533,688,100-byte** complete bundle. Maximum layer plastic strain
is **0.03909795535** (3.910%), cumulative plastic work is **11.19804231 J**, and
221 parents have yielded. The
[physical-scale Chrono video](../../crash-work/renders/source-assembly-native-2ms-1.mp4)
has passed frame inspection, video probing and full decode. The
[accepted bundle](../../crash-work/runs/source-assembly-native-2ms-1/manifest.json)
retains all source mappings, native material histories, groups, contact and
phase-labelled diagnostics. Completion refers to this declared component
horizon, not the 200 ms full-vehicle target or a settled residual shape.

Independent native layered recurrence and large-rotation qualification are
complete within their declared domains. An explicit native shell-frame/normal
rotation policy replaces the unsuitable ordinary-shell total-quaternion proxy;
physical spin, torque, inertia, rigid-member rotation bounds and the remaining
material/contact guards are retained. Native agreement does not imply harmless
local spin or establish physical calibration. See the
[layered rotation qualification](../../planning/LAYERED_NATIVE_ROTATION_QUALIFICATION.md)
and [plasticity review](../../planning/PLASTICITY_CORRECTNESS_REVIEW.md).

The connected source increment is implemented: **seven complete parts /
959 shells / 1,093 nodes**, including the whole **63-node bracket PID 2000204**
and original **spotweld WID 2101297**. The bracket adds 44 shells, rather than
only the weld endpoint. The same six rigid groups remain active. Source-unit
TYPE25 property resolution, native connector recurrence, combined nodal mass/
inertia, resident history and joint publication have passed their owning gates.
The seven-part impact completes **0.9765625 ms / 16,384 intervals at 4h**,
with **104 frames / 560,717,430 bytes**. Maximum PLA is **2.650%**, plastic work
**4.400 J**, and 120 shell parents have yielded. All frames authenticate in the
owning reader; twelve altered-archive cases and 32 affected legacy CUDA functions
also pass. The original weld has sampled peak force **638.34 N**, final force
**475.89 N**, and remains active. Its attached 30-node bracket patch departs
from free flight by up to **2.16649 mm**, while remaining elastic. The separate
33-node source patch remains in exact free flight at every saved sample; neither
bracket patch has saved wall contact. External vehicle supports remain released.
The [independent load-transfer report](../../crash-work/reports/source-seven-part-load-transfer-final-1.md)
records timing, signed work, source identities and the limits of sparse samples.

The [complete seven-part overview](../../crash-work/renders/source-seven-part-native-1ms-1.mp4)
and [labelled detail view](../../crash-work/renders/source-seven-part-native-1ms-detail-1.mp4)
pass image/hash, probe/full-decode and visual checks. The detail enlarges a crop
of the same rendered image by 2x; physical deformation remains 1x, with original
per-frame time and color legend retained. Here `h = 2^-26 s`; seven-part contact
admits 4h and rejects 8h under its unchanged certificate. This completes the
requested four-step milestone; the full vehicle deliverable remains outstanding.

## Current delivery cadence

| Next milestone | Work and ownership | Exit evidence |
| --- | --- | --- |
| Reduce measured step cost | Refresh the short profile for current seven-part/native-geometry execution, then parallelize wall preparation and source-ordered Q/T gather in their TL owners. Reuse timers and bounded arenas. | Exact accepted field/history/ledger parity, failure ordering, late rollback/retry and resource caps pass before a measured speedup is claimed. See the [performance plan](../../planning/PERFORMANCE_NEXT_INTEGRATION.md). |
| Qualify connector-aware comparison | Extend the existing comparator with connector phase, relative motion, force/couple, signed work and combined M/J. Its current guard explicitly rejects seven-part archives because the old metrics are shell-only. | Meaningful positive/negative fixtures and actual seven-part comparisons pass without treating sparse force samples as exact impulse or mixing midpoint and endpoint values. |
| Assess timestep sensitivity at useful physical times | Use the qualified connector-aware comparator and selected smaller-step runs where local response or a new contact event requires them. | Report matching physical-time positions, native layer stress/PLA, cumulative work and complete interval contact events. Preserve phase distinctions, local extrema and unmatched terminal times; do not infer convergence from matching totals alone. |
| Extend toward longer connected crushing | Advance to roughly 2 ms, then 5 ms only while the admitted mechanical/contact domain remains valid. Add the next required complete source connection or contact capability when it becomes limiting. | Diagnosed source-local limits, accounted load paths, stable accepted histories and a new viewable result. Do not relax a guard merely to reach a longer duration or omit entities to fit capacity. |

Independent source semantics, numerical review and reader work can proceed in
parallel. Serialize heavy builds, simulations and rendering. Preserve concise
integration checkpoints and batch the relevant owning regressions there; repeat
long runs only when a change, failure or unresolved physical question warrants
it. The [assembly engine design](../../planning/SOURCE_ASSEMBLY_WALL_ENGINE.md)
and [pilot assessment](../../planning/SOURCE_ASSEMBLY_PILOT_ASSESSMENT.md) contain
the implementation and comparison details.

Stored midpoint kinetic energy, optional force-stage kinetic observations,
endpoint geometry and native work have distinct timing and meanings. Connector
kinetic subtotals are already included in total nodal mass/inertia accounting;
they must not be added twice. Native connector work remains separate from shell
and stabilization work. Constraint reaction work and all stabilization work are
not automatically dissipation. No arbitrary global energy threshold substitutes
for these distinctions. See the
[phase-explicit observation design](../../planning/SOURCE_ASSEMBLY_COLLOCATED_OBSERVATION.md).

## Architecture and reuse

| Owner | Responsibility and retained boundary |
| --- | --- |
| TL-FEA `lib_src/elements`, materials and constraints | Native QEPH/T3 reference/history arithmetic, layered material updates, rigid groups and TYPE25 connector mechanics. OpenRadioss supplies independently pinned reference code and qualification; it is not a production solver runtime. |
| TL-FEA `lib_src/solvers` and collision | Sole CUDA physical-state owner, native mass/inertia, force assembly, contact, temporal scheme and common commit/rollback. No second clock or independent participant advance. |
| robo-dyna `modelio`, `case` and `output` | Authenticated source inventory, explicit policies/boundaries, immutable startup composition, thin execution loop, accepted archives and diagnostics. Reuse shared utilities; keep modules focused and source mappings complete. |
| Chrono and robo-dyna replay adapters | Reuse mesh, geometry, scene and rendering infrastructure. Read accepted TL state at its recorded phase; replay does not run another dynamics solver. |

The active binding, typed shell arenas and mesh-wall contact path now support
explicitly bounded **1,024-parent / 2,048-node** collections, including the actual
six- and seven-part inventories. Legacy defaults of 128 remain compatibility
settings, not the current component limit. Each owner retains independent count
and byte admission, preallocated stepping storage, failed-readback atomicity and
last-entry rollback tests. This is not whole-vehicle capacity qualification.
See [collection scaling](../../planning/SHELL_COLLECTION_SCALING.md) and
[module contracts](../../planning/MODULAR_ARCHITECTURE.md).

## Remaining vehicle milestones

1. **Accounted connected structures:** qualify appreciable transfer through the
   original weld and subsequent required ties, welds and attachments. Preserve
   original cards, complete selected parts and explicit released interfaces.
2. **Folding and broader contact:** add the needed shell self-contact, thickness,
   feature/edge handling, adjacency exclusions, friction and moving footprint
   coverage. The current finite planar mesh-wall domain does not cover general
   vehicle folding contact.
3. **Whole-model mechanics readiness:** resolve the actual required shell, solid,
   beam, nonmetal, failure, joint, mass and other source semantics. Reconcile
   mass/COM/inertia and all load paths before whole-vehicle startup; unsupported
   active mechanics cannot disappear behind a geometry import.
4. **Measured vehicle execution:** qualify state, element, connector, contact and
   output capacities independently. Grow representative workloads, retain
   numerical parity gates and measure bytes/entity, step time and peak resources.
   Extend accepted vehicle horizons toward 20, 50, 100 and 200 ms only as the
   applicable mechanics and contact gates pass.
5. **Delivery and replay:** provide repeatable case setup, complete accepted
   bundles, selected refinement evidence, qualified restart when introduced,
   and an inspected full-vehicle crash/deformation video.

Each new mechanism needs an independent numerical/source reference where
available, a physically meaningful loaded case, source/capacity rejection,
whole-step rollback and exact retry, plus the affected owning integration tests.
Keep the [vehicle design](../../planning/YARIS_RIGID_WALL_DESIGN.md) and
[test catalog](../../planning/YARIS_TEST_GATES.md) as detailed references, with
this roadmap and the active plan determining current priority. Source keyword
counts are not feature counts, and historical coupon tolerances do not become
universal acceptance rules for the vehicle.

## Historical evidence retained

These records describe earlier bounded milestones. Their old capacities,
resource allowances, pending-work statements and test totals are historical.

| Record | What it established |
| --- | --- |
| [Original model audit](../../crash-work/reports/yaris-progress-audit-1.json) | Canonical geometry: 393,165 nodes, 358,457 shells, 15,234 solids, 4,685 beams and 919 parts. Geometry import alone did not admit vehicle mechanics. |
| [Rotational foundation](../../crash-work/reports/rotary-foundation-checkpoint.json) and [Q4 force checkpoint](../../crash-work/reports/q4-force-checkpoint.json) | Earlier small prescribed/coupon force, rotation and transaction gates, with their original restricted domains. |
| [Elastic/contact checkpoint](../../crash-work/reports/elastic-contact-foundations-checkpoint.json) | Supplied-mass contact rigs and restricted elastic/contact foundations; these were not full deforming-vehicle results. |
| [Original-part plastic video](../../crash-work/renders/source-part-plastic-wall-h-1.mp4) and [sensitivity assessment](../../planning/SOURCE_PART_PLASTICITY_SENSITIVITY.md) | Complete 117-node / 94-shell part response through 7.8125 ms, separation and irreversible history. Local refinement differences prevent a blanket convergence claim. |
| [Earlier six-part prefix video](../../crash-work/renders/source-assembly-plastic-prefix-1.mp4) | Accepted response through the former approximately 0.513 ms total-quaternion stop; superseded for current horizon/domain claims by the native-rotation qualification and 1.953125 ms result above. |
| [Shell scaling review](../../planning/YARIS_SHELL_SCALING_REVIEW.md) | Earlier formulation/performance experiments, not a forecast of current whole-vehicle throughput. |

Immutable runs and failed reports remain preserved. The version history of this
file retains the former detailed backlog; do not treat its superseded next-step
statements as active requirements.

## Workstation and publication limits

The user permits autonomous local branches, commits and workspace directories;
**no pushes**. Preserve unrelated repository changes and frozen evidence.
Use the shared `crash-work/reports/workstation.lock` and
`Total-Lagrangian-FEA/tools/run_bounded.py` for heavy work:

- Numerical runs: **2 affinity CPUs**, with declared stage-specific RAM/VRAM caps.
- Builds: **8 affinity CPUs / 4 compiler workers**, **18 GiB** process-group RSS guard.
- Rendering: **4 affinity CPUs**, bounded frame/output budgets.
- Overall user RAM ceiling: **20 GB**; retain at least **32 GiB available host
  RAM** and **8 GiB free VRAM**. Small host-only author checks use their separately
  assigned lower limits and do not run GPU work.

Forecast active storage and complete archive/ledger costs before admission;
frame count alone does not bound a long interval ledger. Stop at a numerical or
resource failure, preserve the accepted prefix, and diagnose its exact source
entity. Performance changes must preserve the qualified mechanics and accepted
results before their speedup is promoted. Refresh this roadmap when the next
capability or completion boundary changes.
