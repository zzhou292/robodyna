# Robo-dyna: Yaris delivery plan

Updated 2026-09-09 after finite Q4 wall coverage, CUDA contact transaction tests,
prescribed batch measurements and the full source-part inventory. This is the
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
This is an engineering judgment about integrated capabilities, with substantial
uncertainty. Passing tests are not a completion ratio. The guided elastic plate
has completed 200 ms with accepted output and inspected video. QEPH force/history,
resident publication, the native full-recurrence screen and sustained one/two-cell
elastic response through 244.140625 microseconds pass, including h/h2/h4
refinements and 86,016 native/CUDA element intervals. Complete prescribed T3
force/history and standalone resident history/publication now pass on CUDA;
the latter owns 6,216 device bytes and adds eight passing functions at TL
`e9e3e5f`. Shared shell utilities are adopted at `329f441`, with all six sustained
scientific response records matching their preceding baseline. The source-part
wall-contact cost gate and stateless contact contributor on the existing owner
also pass. The vehicle is not running yet. Immutable mixed Q4/T3 mass and
identity binding passes eight host tests at TL `9be796a`. Joint resident
publication now passes eight CUDA functions and three actual-owner identity
functions at `e215665`: one five-node union, one kinetic ledger, 20 native cell
interval checks and 23,551 explicit device bytes across nine allocations.
Ten native recurrence-helper tests also pass; all 36 prior native matrices and
six sustained GPU response records preserve their scientific fields exactly.
Mixed force-feedback dynamics remains open. Short shell/wall transactions pass
four CUDA functions at TL `1611802`, including both failure orders and retry;
this is a 0.2384-microsecond preload check. Combined long-response and incoming
impact admission remain open. Native T3/mixed wall contact now passes five CUDA
functions and 41 regressions at TL `3b25699`, including 16 contact-only intervals
with native structural masses, distinct contact weights and late-failure retry.
The contributor keeps one 99,384-byte allocation and its two-parent/eight-incident-
node cap. Mixed shell force feedback remains the next composition gate. The
[combined admission design](../../planning/QEPH_WALL_COUPLED_ADMISSION.md)
separates short transaction checks, contact recurrence, incoming-velocity startup
and impact/refinement; free-shell stability does not qualify wall impact. Plasticity,
connections, self-contact, connected capacity and vehicle output remain major
work. The [current checkpoint](../../planning/EXECUTION_STATUS.md) records
retained scope and limitations; the following tables preserve the earlier
foundation and the broader implementation backlog.

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

## Immediate work packages

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

The current batch kernels report **255 registers and 9,776/9,808 bytes of stack
per thread** in the saved `elastic-coupon-kernel-resources-1.log`. Their two-element
run takes roughly 4.3 ms per accepted step and adds about 2.6–2.7 GiB of device-wide
memory while explicitly owning only 17 KiB. These measurements do not establish
batch occupancy or identify all context/driver/stack allocation. The
[scaling review](../../planning/YARIS_SHELL_SCALING_REVIEW.md) records completed
profiling and prescribed 2/8/32/128-element measurements; next are scratch reduction
and deterministic shared-node assembly before source-part scaling. Keep existing
force/energy oracles unchanged. Compute Sanitizer is not installed in the local
CUDA toolkit, so no sanitizer-clean claim is made.

Grow representative element/contact batches geometrically before actual full
initialization; report bytes/entity, total peak owned memory, milliseconds per
accepted step, candidate counts and host/device transfers. Keep state resident
between output frames. A larger-model run requires a measured memory forecast
that fits the workstation reserves; no GPU speedup or calendar claim yet.

Pinned glslang, VSG and its companion dependencies are now built in an isolated
workspace prefix. A recorded one-line disabled-Assimp fallback include correction
is retained separately from the original archive. The fresh Chrono core+VSG build
is active. The actual rig and elastic coupon frames and the decoded coupon video
have been inspected; the guided-impact and vehicle rendering gates remain open.

All builds and runtime probes use `run_bounded.py` and the shared
`crash-work/reports/workstation.lock`: at most two CPU affinity slots, one build
job, numerical thread pools of one, at least 32 GiB available RAM and 8 GiB free
VRAM. Use smaller per-probe limits and serialize GPU/build work. Parallelize
source implementation, independent CPU audits and review. Stop at guard or
numerical failure and preserve the accepted checkpoint. No pushes.

At each exit artifact, refresh this backlog, capability report and resource
forecast. Reassess progress from the next runnable case and remaining model
coverage; do not increase it for renamed files or additional fixture counts.
