# Robo-dyna: Yaris delivery plan

Updated 2026-09-09 after the complete prescribed CUDA Q4 force gate. This is the
active implementation backlog. The workspace
[architecture and milestones](../../planning/YARIS_RIGID_WALL_DESIGN.md),
[module contracts](../../planning/MODULAR_ARCHITECTURE.md) and
[test catalog](../../planning/YARIS_TEST_GATES.md) provide the detailed contracts.

The first vehicle deliverable remains the original **2010 Yaris coarse V1l at
35 mph into a stationary finite triangle-mesh wall, through 200 ms**, using
TL-FEA CUDA mechanics and Chrono infrastructure. Robo-dyna's longer-term goal
is LS-DYNA-like CAE functionality. No external production solver is introduced.

**Current assessment: approximately 10–20% toward the vehicle deliverable.**
This is an engineering judgment about integrated capabilities, with substantial
uncertainty. Passing tests are not a completion ratio. Package A now adds a
complete prescribed elastic Q4 force operation; rotary dynamics and an actual
deforming trajectory remain the next mechanics gates.

## Verified starting point

| Capability | Fresh evidence and practical boundary |
| --- | --- |
| Original vehicle geometry | All 17 canonical arrays hash correctly: 393,165 nodes, 358,457 shells, 15,234 solids, 4,685 beams, 919 parts. The actual importer rejects simulation admission: `geometry-only model`. |
| Finite mesh wall and accepted output | Both retained 70 ms runs verify all 37 artifacts each. Each contains 140 accepted CUDA steps and 15 Chrono mesh frames, using supplied mass and contact without shell elasticity. |
| Shell arithmetic | Complete TL prescribed CUDA Q4 forces now pass eight integration tests, including all 24-DOF energy derivatives and two-element shared-node assembly. Ten setup, seven host/CUDA rotation and four assembly tests also pass. These 29 new checks bring the retained total to 305. |
| Application integration | The previous seven application CTest groups pass, including GPU nodal output and normal impact. Existing nodal/contact Bazel regressions remain passing; no force-driven shell trajectory or renderer yet. |
| Dynamics and contact limits | State supports at most 64 translational nodes and rejects angular motion/couples. Contact supports 25 surface nodes/32 triangles, fixed Y/Z, zero offset/friction and a finite fixed footprint. |

Evidence is indexed in the [Q4 force checkpoint](../../crash-work/reports/q4-force-checkpoint.json).
The [preceding live probe](../../crash-work/reports/yaris-progress-audit-1.json)
records the imported model and saved contact rigs. Chrono remains `ab261d4baa`.
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

Package **A is complete within its prescribed elastic rectangle scope**. B–F
remain pending; mechanics B, contact C, source-model E and rendering F can
proceed against separate module contracts.
The first combined artifact depends on their applicable gates, not all Yaris
semantics. Freeze fixture scales and tolerances before declaring a pass.

| Package | Implementation and dependency | Required tests and exit artifact |
| --- | --- | --- |
| **A — Complete prescribed CUDA Q4 forces** (M2 subset; **passed**) | TL `ReissnerShellData`, `ReissnerRotation`, `ReissnerShellKinematics`, `ReissnerShellForce` and `ReissnerShellAssembly`, plus the narrow `ReissnerShellSetup` host adapter. Actual Chrono setup and centered isotropic elastic section arithmetic; no new state owner. | All 29 new tests pass. CPU/CUDA strains, dimensionally scaled resultants, world forces/couples and energy; all 24-DOF energy derivatives; common finite rotation and initial-frame offsets; two actual Q4s sharing physical nodes; failed publication and clean retry. No mass, rotary stepping, tangent or source Yaris formulation is qualified by this gate. |
| **B — Rotational state and elastic dynamics** (M3/M4a subset) | Extend the existing owner with optional quaternion/world angular velocity and force/couple assembly, per-component translational constraints, and declared rotary inertia. Separate stepping operation. Compose a no-contact elastic patch after A. Retain fixed-step velocity-first stepping for this first coupon. | Independent mass/thickness inertia integrals; arbitrary-axis spin/torque, quaternion norm and covariance; constraint reactions; shared-node mass counted once; late failure leaves all accepted state/output intact; nonuniform bending and h/h2/h4 refinement. Exit: accepted deforming patch frames with force, strain and energy histories. |
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
A sampled quad has 0.185-degree warp; this does not qualify the entire part.
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

The first force-check kernel reports **255 registers and 9,104 bytes of stack
per thread** in the saved `cuobjdump` report. Its one-thread correctness test
does not establish batch occupancy, performance or total local-memory demand.
Measure/tune execution layout before scaling; keep the qualified force/energy
tests unchanged while doing so. Compute Sanitizer is not installed in the local
CUDA toolkit, so no sanitizer-clean claim is made.

Grow representative element/contact batches geometrically before actual full
initialization; report bytes/entity, total peak owned memory, milliseconds per
accepted step, candidate counts and host/device transfers. Keep state resident
between output frames. A larger-model run requires a measured memory forecast
that fits the workstation reserves; no GPU speedup or calendar claim yet.

All builds and runtime probes use `run_bounded.py` and the shared
`crash-work/reports/workstation.lock`: at most two CPU affinity slots, one build
job, numerical thread pools of one, at least 32 GiB available RAM and 8 GiB free
VRAM. Use smaller per-probe limits and serialize GPU/build work. Parallelize
source implementation, independent CPU audits and review. Stop at guard or
numerical failure and preserve the accepted checkpoint. No pushes.

At each exit artifact, refresh this backlog, capability report and resource
forecast. Reassess progress from the next runnable case and remaining model
coverage; do not increase it for renamed files or additional fixture counts.
