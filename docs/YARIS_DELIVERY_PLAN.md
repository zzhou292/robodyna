# Robo-dyna: Yaris delivery plan

Updated 2026-09-09 after original source-part elastic refinement, accepted video
and measured parallel CUDA speedup. This is the
implementation backlog. The latest measured ordering is maintained in the workspace
[active execution plan](../../planning/CURRENT_EXECUTION_PLAN.md). The workspace
[architecture and milestones](../../planning/YARIS_RIGID_WALL_DESIGN.md),
[module contracts](../../planning/MODULAR_ARCHITECTURE.md) and
[test catalog](../../planning/YARIS_TEST_GATES.md) provide the detailed contracts.

The first vehicle deliverable remains the original **2010 Yaris coarse V1l at
35 mph into a stationary finite triangle-mesh wall, through 200 ms**, using
TL-FEA CUDA mechanics and Chrono infrastructure. Robo-dyna's longer-term goal
is LS-DYNA-like CAE functionality. No external production solver is introduced.

**Current assessment: approximately 25–30% toward the vehicle deliverable.**
This is a capability-based engineering estimate, not a test-count or elapsed-time
ratio. The vehicle is not running. The
[original source-part elastic video](../../crash-work/renders/source-part-elastic-h-video-1/yaris-part-elastic.mp4)
now shows accepted deformation of the actual Yaris geometry. Its 258 frames
cover 1.953125 ms of simulation over 10.32 s of playback, with displacement
explicitly magnified 25x and an immutable original reference outline.

All 117 nodes, 88 Q4 and six T3 of PID 2000157 run in one app-owned
`SourcePartElasticCase` at app `25434d0`. The complete h/h2/h4 pulse/free-response
study passes all six comparison fields at 257 shared physical times; maximum
chord-length change is about 0.277 mm. The short actual-source native gate covers
6,016 cell intervals, including late material/observer rejection and exact retry.
Accepted archives, strict replay and Chrono rendering pass. This is the explicit
free-part LAW1 elastic experiment; original MAT024 behavior, six nodal-rigid
groups and tied attachments are not applied. It is not yet a part-wall impact.

The CUDA mechanics foundation now includes complete prescribed QEPH/T3 elastic
force/history, actual native structural mass/inertia, one shared nodal owner,
and atomic joint publication. Sustained one/two-cell free response passes.
Full broadside finite-mesh wall response also passes all six runs and both
h/h2/h4 comparisons through 244.140625 microseconds (86,016 native/CUDA cell
intervals). TL source `b5811eb`, results `4e6dc1d`; the coarse synchronous
energy residual is below 3.855e-7 of initial kinetic energy. This named fixture
translates and rebounds rigidly and establishes no deforming vehicle result.

Mixed Q4/T3 feedback now passes at TL `4650560`: after a finite pulse, both
nonzero native force caches drive free steps on the same five-node owner.
Three new CUDA functions and 42 affected regressions pass, including late
failure in either family and exact retry. The production change preserves
force/history arithmetic and allocation; no external solver runtime is added.

Robo-dyna's accepted mesh adapter now explicitly handles staggered timing and
retains actual endpoint x/q, midpoint v/omega and complete accepted stamp.
Two new and five existing CUDA functions pass; failed mesh publication preserves
all exposed fields. The source-part case now uses that path for accepted
archives and the inspected video without reconstructing missing physical fields.

The retained total is **1,142 distinct passing functions**. Detailed historical
results, limitations and exact logs live in the [execution checkpoint](../../planning/EXECUTION_STATUS.md).
The capacity architecture keeps one immutable shared binding, two typed element
batches, one state owner/coordinator and the existing finite-wall contributor.
TL `efab23a` now qualifies 128-node/parent capacity through the existing owner,
batches and finite-wall contributor. Three resident and five contact CUDA/host
functions pass with affected regressions and six owning Bazel targets. Both
complete coarse rebound runs preserve every scientific field exactly after the
storage change. The actual source-part elastic experiment now retains all 117
original nodes and 94 original shell parents. Source MAT024, attachments,
self-contact, beams/solids/connectors and full-vehicle output remain major gates.
No source entity may be silently removed to satisfy a fixture limit.

The complete source-part host binding passes: all 117 original nodes, 88 Q4
and six T3 retain exact geometry, source IDs and ordering. Native structural
mass is 0.25650893888187326 kg; total rotary inertia is 5.1937904054349167e-6
kg*m^2, with physical/added partitions retained separately. Two new tests and
nine affected original-source regressions pass. This is startup for the explicit
elastic experiment. Resident capacity and the frozen actual-part elastic
trajectory/refinement are qualified within that scope.
Evidence: `crash-work/reports/source-shell-collection-tests-1/`.

Parallel parent evaluation preserves all 778 scientific files / 42,954,314 B of
the complete coarse source run exactly. Runtime falls from 331.503 s to 99.327 s,
a 3.33749x speedup, with unchanged owned device allocation and ordered reductions.
The quarter-step run completes in 390.55 s with 184,127,488 B peak sampled RSS.
Twenty existing QEPH functions and owning Bazel gates pass after the parallel
change, retained locally at TL `c419359`. These are measured part results, not vehicle-scale
throughput. No new test count is assigned to repeated runs or refinement.

TL `7335646` / app `f746d39` now qualify shared moving startup/common K0,
source/placed-wall geometry, actual-source uniform free flight and the contact
helper: six + five + three + six new functions, with affected regressions and
owning build gates. The [preparation checkpoint](../../crash-work/reports/source-wall-preparation-checkpoint-1.json)
pins this 20-function increment and the frozen impact pilot. Coupled source-part
contact dynamics have not run yet.

## Current delivery cadence

The user requested faster progress and periodic demo videos on 2026-09-09.
The original elastic source-part milestone now passes, including refinement and
video. Extend the same case to a gentle impact against the actual finite mesh
wall; preserve its existing owner, native binding, typed batches, common
publication and accepted-output path.

1. Incoming startup, geometry, actual-source free flight and the contact helper
   are integrated and qualified at TL `7335646` / app `f746d39`. Reuse their
   native mass/K0 authentication, certified contact areas and explicit wall
   placement in the same case.
2. Qualify one actual-source contact-onset/failure-retry integration. Follow the
   [frozen wall pilot](../../planning/SOURCE_PART_WALL_PILOT.md): 1 m/s, original
   0.5 mm gap and 8,448-H native-checked prefix through 503.5400390625 microseconds.
   Check all 94 native histories and independently evaluated accepted-base
   contact, then late rejection/exact retry. The full pilot retains its about
   2 ms initial horizon, native-K0/area-based penalty and separate timestep guards.
3. Run the same impact at h/h2/h4 with synchronized energy, wall impulse,
   strain/curvature, area/thickness, coverage and penetration checks before every
   commit. Archive the actual placed 62-vertex/100-triangle wall, inspect and
   fully decode a new video, and require documented separation/rebound before
   calling the result a completed impact.

Reaction comparison uses the frozen scale `2*K0/design_penetration`, about
684.024 N. Physical energy requires absolute residual plus uncertainty to stay
within `0.05*K0 + 1e-10 J`; uncertainty does not enlarge that physical budget.

Reuse NodalMeshOutput, ArtifactIO/Inventory/MeshArchive and the VSG pipeline.
The impact video needs exact source mappings, native mass/inertia, accepted
positions/orientations, correctly timed velocities and work diagnostics. Full
private-history/restart export is optional; it must not delay that result.
The complete part uses 117 vertices and 182 display triangles. Reference
oracles stay in the short qualification gate, not the production timestep.

Batch focused regressions at integration checkpoints and retain local commits,
logs and required input bindings. Extra broadside variants and reporting
frameworks are deferred unless a failure needs them. Parallelize source work;
serialize heavy builds/GPU runs. Builds use two workers on four affinity CPUs,
an 18 GiB RSS guard, and remain below the user's latest 20 GB RAM ceiling.
Numerical jobs use one affinity CPU and lower per-stage limits; retain at least
32 GiB available RAM and 8 GiB free VRAM. No pushes.

## Historical verified starting point

| Capability | Fresh evidence and practical boundary |
| --- | --- |
| Original vehicle geometry | All 17 canonical arrays hash correctly: 393,165 nodes, 358,457 shells, 15,234 solids, 4,685 beams, 919 parts. The actual importer rejects simulation admission: `geometry-only model`. |
| Finite mesh wall and accepted output | Both retained 70 ms runs verify all 37 artifacts each. Each contains 140 accepted CUDA steps and 15 Chrono mesh frames, using supplied mass and contact without shell elasticity. |
| Shell arithmetic | Complete TL prescribed CUDA Q4 forces now pass eight integration tests, including all 24-DOF energy derivatives and two-element shared-node assembly. Ten setup, seven host/CUDA rotation and four assembly tests also pass. These 29 checks preceded B1 and brought the retained total to 305 at that checkpoint. |
| Application integration | The previous seven application CTest groups pass, including GPU nodal output and normal impact. Existing nodal/contact Bazel regressions remain passing; B2 passes the half-period/refinement, artifact, replay/scene and actual Vulkan capture/video checks. |
| Rotational foundation (B1) | Fourteen actual CUDA tests pass for isotropic spin/torque, world constraints, reactions and transaction failures; ten host mass/inertia tests pass. One added Chrono output test passes. Retained total: 330 distinct checks; reruns are not added. |
| Dynamics and contact limits | One state owner supports at most 64 physical nodes, optional quaternions/world angular velocities and declared isotropic inertia. Rotational stepping now also admits a separately declared restricted elastic trajectory only after a matching candidate receipt; coupled contact remains unqualified. Existing contact remains limited to 25 surface nodes/32 triangles, fixed Y/Z, zero offset/friction and a finite fixed footprint. |

Foundation evidence is indexed in the [rotational foundation checkpoint](../../crash-work/reports/rotary-foundation-checkpoint.json);
the [Q4 force checkpoint](../../crash-work/reports/q4-force-checkpoint.json) remains frozen.
The [preceding live probe](../../crash-work/reports/yaris-progress-audit-1.json)
records the imported model and saved contact rigs. Chrono is now `96af26597b`,
with checked VSG worker lifecycle and unrelated third-party dirt preserved.
The [actual R0/R1 checkpoint](../../crash-work/reports/vsg-r0-r1-runtime-checkpoint-1.json)
records rendered rig and coupon frames. The
[elastic/contact foundations checkpoint](../../crash-work/reports/elastic-contact-foundations-checkpoint.json)
retains matching binaries, libraries and accepted evidence. TL `8f781ce`
retains finite Q4 footprints; `4de53e5` retains measured prescribed batching.
Historical failed native rigid-motion and unmodified Chrono/CCD experiments
remain excluded from production qualification.

## Ownership and reuse

| Owner | Work to add or extend | Reuse boundary |
| --- | --- | --- |
| TL `lib_src/elements` | Complete conventional Q4 force/energy operation, compact immutable reference data, later mixed element families | Reuse qualified Chrono force/rotation arithmetic, TL quadrature and existing CUDA assembly. Do not allocate a CPU element object per GPU element. |
| TL `lib_src/solvers` | Optional rotational state, component constraints, declared inertia and checked stepping | Extend `FENodalState`, its views and accepted/trial transaction; retain one owner, one force reset and one commit per step. |
| TL `lib_src/materials` and existing section abstractions | Source-qualified elastic/plastic/rate/nonmetal behavior as needed | Use existing Chrono/OpenRadioss material mathematics with provenance and independent gates. Elastic Q4 initially uploads a validated constant section matrix from Chrono setup. |
| TL `lib_src/collision` | FE parent interpolation, contact integration, motion admission, broadphase integration, later thickness/friction/self-contact | Extend tested geometry, normal law, mass/Jacobian and finite-wall ownership. `CudaSurfaceContactSystem` is a proposed composition, not an existing implementation. |
| Chrono `src/chrono/fea` and core | Qualified numerical reference; reusable setup, mesh, archive and later scene APIs | Preserve the default-off coherent reference and existing behavior. Chrono output reads accepted TL state and does not advance another dynamics clock. |
| `robo-dyna/tools`, `case`, `chrono` | Typed model compilation, small case composition, narrow setup/output adapters and result bundles | Extend existing importer, `NormalImpactCase` patterns and `NodalMeshOutput`; keep mechanics in TL. Split importer semantics into cohesive modules as they are implemented. |

Backend unit tests live in TL `lib_utest`; Chrono reference tests and application
integration tests stay with their owning modules. No growing omnibus prototype
or collection of per-experiment production scripts.

## Historical work packages and broader backlog

The records below preserve the earlier component progression and its original
limits. The current cadence above supersedes their next-step statements;
accepted and failed numerical evidence remains unchanged.

C2 integration passes 13 host/five CUDA tests and Bazel, retained in TL
`da4516b`. C3 passes nine finite-mesh host checks and Bazel. C4 now passes eleven
CUDA transaction checks and five host contact-stiffness checks through CMake and
Bazel, including error handling before force publication and exact-coordinate
area certificates; TL commit `5dc521b` retains this contribution.
D1 passes six guided reference tests and seven combined-admission tests, with
the eight existing B2 reference checks unchanged. The guided case proposes
19,997 steps through 0.2 s at approximately 10 microseconds per step. Its
100-step coupled CUDA prefix and all six D2 transaction functions now pass,
including the explicitly enabled partial-contact/retry test. First certified
applied contact occurs at epoch 12,365 / 123.668550 ms. The tested prefix reaches
1.354053 micrometers peak penetration; it does not cover the full impact.
Accepted bundle support now passes seven writer and seven reader checks, with
one added scene check and passing legacy output regressions. Six host wall
transformation tests cover original/flipped/subdivided meshes, explicit derived
provenance and prescribed-state contact invariance. Thirteen study, six report
IO and three small derived-wall case tests pass. The first full-h attempt
reached saved epoch 12,500 / 125.019 ms before the 240-second guard stopped it.
No completed bundle, rebound or refinement is claimed. Prescribed profiling
finds thousands of serial C2 leaves at partial contact, with about 168–402 ms
per active-parent CUDA evaluation. The direction-selective dyadic candidate now
passes twelve host/four CUDA functions and all three pinned saved-state profiles:
both partial-contact states use eleven leaves, taking about 1.116/1.749 ms on
the GPU with unchanged geometry and error/capacity budgets. Its opt-in C4 path
passes all 23 owner-test executions (twelve distinct functions), with scalar
remaining the default and one selected bounded allocation. Application backend
identity, Study/report and accepted replay wiring now pass, as do both long
first-contact checks. The rectangular full-h run then reached the unchanged
0.5 mm penetration stop at 145.732 ms in 78.015 seconds, with peak energy error
0.017925% and remaining inward kinetic energy. Its incomplete archive is
preserved. A named penalty-margin-v1 experiment now precedes the next full
h/h2/h4 impact, wall-response studies and rendering: higher stiffness is
explicitly identified and must pass a nonlinear prescribed energy audit;
absolute integration errors and stop limits remain unchanged. The original
physical experiment remains reproducible. Backend choice stays an independent
configuration identity, including recorded axis-depth conventions.
See [the contact execution review](../../planning/GUIDED_CONTACT_EXECUTION_REVIEW.md).
See the [guided plate implementation](../../planning/GUIDED_PLATE_IMPLEMENTATION.md).
Parallel E1 passes 12 synthetic declaration tests and the original PID 2000157
closure compilation; it does not yet supply geometry/mass/attachment admission.
P1's separate 2/8/32/128-element processes measured the unchanged force operation:
about 0.969–0.993 ms for force evaluation and 2.043 GiB context-to-evaluated
device-memory growth, despite at most 645,640 explicitly owned bytes.
Five qualification tests pass; these are prescribed forces, not vehicle steps.
P2's ANS transverse-row optimization passes eleven new numerical checks and
isolated batch measurements. It reduces the measured warm force-event medians
by about 5%, with only 16 bytes less stack per thread and 6 MiB less observed
device-memory growth. Immediate Gauss contraction now also passes eleven new
CUDA parity checks and the same bounded wrapper tests. Its force stack is
6,544 bytes, with 1,447,034,880 bytes of measured device-wide growth; event
medians are 9.36–10.16% lower than the earlier scalar comparison. TL commit
`9c9d4f1` retains both experiments; production dynamics remain unchanged.
See the [stage-2 evidence](../../planning/SHELL_P2_GAUSS_CONTRACTION_DESIGN.md) and
[scaling review](../../planning/YARIS_SHELL_SCALING_REVIEW.md).

Package **A and the B1 state/mass foundation are complete within their stated scopes**.
B2 has passing modal, CUDA batch, full half-period/refinement, output and
replay/scene checks and actual graphical/video exit. Contact C,
source-model E and rendering F can
proceed against separate module contracts.
The first combined artifact depends on their applicable gates, not all Yaris
semantics. Freeze fixture scales and tolerances before declaring a pass.

| Package | Implementation and dependency | Required tests and exit artifact |
| --- | --- | --- |
| **A — Complete prescribed CUDA Q4 forces** (M2 subset; **passed**) | TL `ReissnerShellData`, `ReissnerRotation`, `ReissnerShellKinematics`, `ReissnerShellForce` and `ReissnerShellAssembly`, plus the narrow `ReissnerShellSetup` host adapter. Actual Chrono setup and centered isotropic elastic section arithmetic; no new state owner. | All 29 new tests pass. CPU/CUDA strains, dimensionally scaled resultants, world forces/couples and energy; all 24-DOF energy derivatives; common finite rotation and initial-frame offsets; two actual Q4s sharing physical nodes; failed publication and clean retry. No mass, rotary stepping, tangent or source Yaris formulation is qualified by this gate. |
| **B1 — Rotational state and shell inertia** (M3 subset; **passed**) | Extended `FENodalState`; separate `ExplicitNodalStep`; shared element-independent quaternion utility; `ReissnerShellMass` with separate physical/numerical inertia. Six startup device allocations in either state mode, one accepted/trial commit. | 14 CUDA nodal, 10 host mass and one added Chrono output checks pass. Arbitrary-axis and noncoaxial spin/torque, exact impulse/discrete work, world-axis constraints and base-time reactions, shared-node accounting, whole-state rejection/retry and validator CUDA failure before commit. **Prescribed constant loads only; no deforming shell trajectory.** |
| **B2 — No-contact elastic dynamics** (M4a subset; numerical/output/rendering gates pass) | Compose two actual Q4s/six nodes with A+B1, a clamped short edge and a small modal initial bend. Add a separately qualified coupled admission; keep immutable rest geometry and declared inertia. Detailed [elastic coupon design](ELASTIC_COUPON_DESIGN.md). | All-DOF modal/finite-difference cross-checks, wave/rotary timestep scales, bounded candidate chart/geometry/energy envelope, nonuniform bending, h/h2/h4 and whole-step rollback. Exit: accepted deforming patch frames with force, strain, physical/artificial energy histories and actual rendering. The constant-load declaration cannot be used for shell forces. |
| **C — Q4 contact against the finite wall** (Contact subset) | Independently qualify prescribed Q4 midsurface contact. Reuse finite wall triangle ownership and normal law; add physical parent IDs, parametric coordinates, Q4 interpolation and quadrature. Initially fixed projected footprint, zero offset and friction. Permit director rotation under that explicit contract. | Point position/velocity, resultant, moment and virtual work; smooth force/potential derivatives; fully active analytic fields; partial activation against independent refined integration; reversed wall diagonals, seams and subdivision; prepared-motion rejection and retry. Exit: force/scatter and partial-contact integration checkpoint. |
| **D — Guided deforming mesh-wall plate** (**M5a.1**) | Compose A+B+C through public TL operations and accepted Chrono output. Constrain tangential translations for the declared fixed footprint; allow genuine nonuniform normal displacement and director rotation. Use the actual canonical finite wall. | Different contact activation times across the plate, measurable bending, wall reaction/impulse, structural/contact energy ledger, candidate chart/depth validation, h/h2/h4 and wall refinement, rejection after a previously accepted step. Exit: viewable accepted frames and complete diagnostics. This is a restricted intermediate gate; full M5a remains open. |
| **E — Source-derived part readiness** (parallel M1 subset) | Compile typed sections, MAT024 inputs/curves, units/defaults, attachment closure and a mass/COM/inertia ledger for PID 2000157. Audit all its geometry and required connections before choosing a runnable extraction. | Exact IDs/topology, curve/rate units, unknown-option rejection, independent mass, no dropped triangles or attachments; explicit variant manifest for any cut boundary or elastic override. Exit: source-backed capability report and reproducible extraction, even if simulation remains blocked. |
| **F — Accepted-result replay and rendering** (parallel output track) | Reuse Chrono VSG with a bounded result reader, scene adapter and thin optional replay executable. Extract shared archive/hash utilities from the existing output code; keep dynamics out of the viewer. | First replay/render the saved rig, then visibly render the actual elastic coupon and guided impact. Preserve source/part/time mapping and deformation normals; qualify multi-material shapes, capacity and any topology changes. The final delivery includes a rendered crashing/deforming vehicle video. See [R0–R4 rendering gates](RENDERING_ARCHITECTURE.md). |

The current triangle-centroid/equal-three-node contact spring is not a qualified
bilinear Q4 contact map. Display triangulation must not choose FE force weights.
For C, target normalized FP64 interpolation/work error at most `1e-10` on small,
well-conditioned fixtures; set separate integration tolerances for partially
active patches from convergence evidence. Fixed quadrature can miss a small
active region. For B/D, use the catalog's initial at-most-5% change in relevant
observables when halving h, with another refinement if inconclusive. Use an
explicit geometric penetration scale for the zero-offset idealization.

**Rotary inertia and stability decisions:** physical tangential director inertia
is independently integrated as `rho*t^3*A_i/12`; physical drilling inertia is
zero. The first coupon may use the already proposed, explicitly reported
numerical drilling inertia equal to that tangential value. Report physical and
artificial rotational energy separately and test sensitivity. Chrono's heuristic
box inertia is not the physical shell inertia. The current frozen translational
PSD bound does not certify nonlinear shell forces or rotations. B/D must provide
a justified policy for their admitted configuration/trajectory, including
independent modal/step checks and prepared candidate validation. Relative
director chart angles must remain below 90 degrees; never reset reference
geometry to conceal a failed chart. Central difference, variable steps and
general rotary inertia are later qualified extensions when needed.

## Keep an actual Yaris part on the parallel track

The audited **PID 2000157, `177_railfrontconnector`**, contains **88 Q4 and six
T3** elements, source ELFORM2/NIP3, 1.648 mm thickness and MAT024 inputs:
density 7,890 kg/m3, E 200 GPa, Poisson ratio 0.3, yield 270 MPa, rate parameters
C=8000/P=8 and hardening curve 2100270. See the saved
[source part audit](../../crash-work/reports/yaris-element-roles.json).
The complete 94-shell/117-node inventory has maximum Q4 warpage of **12.17
degrees** for diagonal 0–2 and **12.56 degrees** for diagonal 1–3, substantially
above the earlier single-quad sample. It has no shared-node structural neighbors,
but six source nodal rigid groups connect 20 of its nodes to 56 external nodes
across five neighboring parts. Membership in a large tied-contact master set
is additional unresolved scope, not a resolved attachment pair. See the
[source-part readiness plan](../../planning/YARIS_SOURCE_PART_READINESS.md).
E2a now verifies every selected original source card and passes the complete
geometry and lamina mass/COM/inertia audit. Its mass proxy is 0.2565425684 kg;
23 geometry/mass/composer tests and six donor-quadrature checks pass. The
[readiness checkpoint](../../crash-work/reports/e2a-readiness-checkpoint-1.json)
retains the evidence. This proxy does not establish source ELFORM2 lumping or
solver admission. E2b now adds typed, bounded one-hop attachment records:
15 new tests pass, original incidence verifies 173 nodes/182 elements, and
independent source review found no blocker within that scope. Six rigid
groups and tied-set membership remain separate namespaces; no tied pairs,
attachment mechanics or full closure are inferred. The retained v2 report
fits the unchanged 1 MiB publication cap with only 16,945 bytes remaining.
The later bumper PID 2000132 includes 69 triangles and a sampled 11-degree
warped quad, so flattening the model is not an acceptable import assumption.

A declared one-Q4 extraction with an elastic override can be an early coupon.
The complete connector part needs native triangle support, admitted reference
warpage, its actual material and accounted-for boundaries. A joined bumper/rail
additionally needs its source connectors/absorbers. Do not equate an extraction
with completion of the original subassembly milestone.

The full manifest's **3,583 blocking keyword blocks are not 3,583 new solver
features**. Assign explicit supported, equivalent, diagnostic-only,
not-applicable-with-reason or unsupported dispositions to typed semantics.
Output requests can map to declared robo-dyna outputs; LS-DYNA binary formats
are not prerequisites unless required by the deliverable. Active mechanics may
not disappear under that mapping. Record the intentional mesh-wall replacement
of analytic source wall semantics and report each remaining unsupported field.

## Following milestones and promotion evidence

**New gate immediately after the guided impact: source-scale shell formulation
and inertia.** The current rotary step rule is
`h <= 0.1*t/sqrt(12*G/rho)`. Substituting the source connector's 1.648 mm steel
section yields about 15.24 ns, or at least 13.13 million steps for 200 ms,
before any stricter limit. This is a calculation from the current policy,
not a measured vehicle result. It exceeds the current startup step cap and
cannot be addressed by the measured force-kernel improvements alone. The
[source-bound decision and comparison gate](../../planning/THIN_SHELL_EXECUTION_DECISION.md)
records independent arithmetic and the documented donor inertia policies.

Use the completed guided case as an integration reference. Before promoting
source-part dynamics or starting another Reissner microoptimization round,
compare existing explicit crash-shell formulations and rotary-inertia policies
from OpenRadioss and other already available donor code. Preserve physical and
declared artificial inertia separately, reuse TL's owner/contact/output
contracts, and require bending, membrane, rigid-motion, hourglass and timestep
gates appropriate to the selected formulation. A source shell mapping remains
explicit; neither a larger step nor different inertia may be introduced
silently. The completed Gauss contraction experiment remains useful as
reference/performance evidence; it does not change that ordering.

| Milestone | Capability that must be added | Exit evidence |
| --- | --- | --- |
| **M4b and full M5a** | Plane-stress plasticity/hardening/rate history; general shell-wall contact with thickness/offsets, moving footprints, finite edges/corners, friction and qualified motion safety. Develop independently after the relevant module contracts pass. | Material point yield/unload/reload/rate and rollback tests; oblique and boundary impacts, force/couple virtual work, no unhandled crossing, step/mesh refinement. |
| **M5b — Plastic impact** | Couple qualified metal history to general mesh-wall dynamics; admitted source-shell mappings and warpage for the chosen case. | Permanent deformation, nonnegative material dissipation, full energy/history ledger and accepted playback. |
| **M6 — Folding/self-contact** | Both surface directions, edge-edge features, shell thickness, adjacency exclusions, friction history and complete capacity-safe candidates. | Folding rail/box with no dropped contacts, bounded retries, stable seam history and momentum/work checks. Extend the current six-node local stencil before up-to-eight-node Q4/Q4 endpoints are admitted. |
| **M7 — Joined Yaris structure** | Source-extracted bumper/rail with required T3/Q4 formulations, welds/ties, rigid attachments, solids and absorbers. | Strict variant/capability and mass ledger; load transfer through intact connections; crush/force/energy histories and viewable permanent deformation. |
| **M8 — Whole Yaris startup** | All active materials/elements/connections, added masses, rigid groups/joints, include transforms, initial velocities, tires/ground/gas state and output semantics. | Complete mechanics coverage, reconciled mass/COM/inertia, topology/ownership checks, bounded resident initialization and controlled startup. |
| **M9 — Whole crash** | Extend accepted duration through **1, 5, 20, 50, 100 and 200 ms**, identifying actual first contact. | At each horizon: finite state, diagnosed limiting step/entity, impulse/energy/penetration budgets, complete contact and measured resources. |
| **M10 — Delivery** | Repeatable case entry point, phase-correct restart, meaningful playback through Chrono and result export. | Full run bundle, executable instructions, viewable deformation/video, selected refinement and restart-versus-uninterrupted checks. |

The original vehicle requires mixed shell/solid/beam families and nonmetals,
failure, connections and tire cavities. Supporting the first elastic Q4 is not
an automatic mapping of LS-DYNA ELFORM2/16/9 or MAT024 options. Pull required
families forward whenever the next source extraction uses them.

## Scalability and workstation gates

The separately tested TL broadphase is not connected to `PlanarMeshContact`.
It expects column-major coordinates/default-stream execution; the state owner
uses interleaved coordinates/its own stream. Implement a resident adapter with
explicit synchronization and complete batching. Account for mesh, sort,
neighbor, candidate, history, trial and output memory; retain brute-force
candidate oracles, dense/folded scenes and forced-capacity failure tests.
Raising fixture caps alone does not qualify vehicle operation.

There are 336,976 source Q4 and 21,481 T3 shells, producing **695,433 shell
display triangles** before solid surfaces. Canonical arrays occupy only 43.31
MiB, but replicating Chrono's persistent B/D/G matrices at four points for every
Q4 would consume approximately **8.77 GiB for those three caches alone**.
Reuse their arithmetic with compact state and measured scratch. Whole-vehicle
state, contact and output capacity must each be qualified.

The historical elastic-coupon batch kernels report **255 registers and 9,776/9,808 bytes of stack
per thread** in the saved `elastic-coupon-kernel-resources-1.log`. Their two-element
run takes roughly 4.3 ms per accepted step and adds about 2.6–2.7 GiB of device-wide
memory while explicitly owning only 17 KiB. These measurements do not establish
batch occupancy or identify all context/driver/stack allocation. The
[scaling review](../../planning/YARIS_SHELL_SCALING_REVIEW.md) records completed
profiling and prescribed 2/8/32/128-element measurements. The later QEPH/T3
source-part case now has the measured 3.33749x parallel-evaluation improvement
above; whole-vehicle memory, assembly and contact scaling remain open. Keep existing
force/energy oracles unchanged. Compute Sanitizer is not installed in the local
CUDA toolkit, so no sanitizer-clean claim is made.

Grow representative element/contact batches geometrically before actual full
initialization; report bytes/entity, total peak owned memory, milliseconds per
accepted step, candidate counts and host/device transfers. Keep state resident
between output frames. A larger-model run requires a measured memory forecast
that fits the workstation reserves. Do not extrapolate the measured part speedup
to whole-vehicle throughput or a calendar estimate.

Pinned glslang, VSG and its companion dependencies are now built in an isolated
workspace prefix. A recorded one-line disabled-Assimp fallback include correction
is retained separately from the original archive. The fresh Chrono core+VSG build
is available. Actual rig, elastic coupon, guided plate and original source-part
elastic frames/videos have passed their scoped inspections. Source-part wall
impact and full-vehicle rendering remain open.

All builds and runtime probes use `run_bounded.py` and the shared
`crash-work/reports/workstation.lock`: builds use at most four CPU affinity slots,
two compiler workers and 18 GiB process-group RSS under the user's 20 GB RAM
ceiling. Numerical jobs use one affinity CPU, thread pools of one and lower
stage-specific caps, retaining at least 32 GiB available RAM and 8 GiB free
VRAM. Serialize GPU/build work. Parallelize
source implementation, independent CPU audits and review. Stop at guard or
numerical failure and preserve the accepted checkpoint. No pushes.

At each exit artifact, refresh this backlog, capability report and resource
forecast. Reassess progress from the next runnable case and remaining model
coverage; do not increase it for renamed files or additional fixture counts.
