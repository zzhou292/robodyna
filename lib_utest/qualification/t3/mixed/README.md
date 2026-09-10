# MB1: joined native shell publication

**Source contract; no mixed runtime pass yet.** This gate joins one resident
QEPH Q4 and one resident native T3 through one `FENodalState`. It qualifies
prescribed state/history transactions, not force-feedback dynamics, a stable
mixed timestep, contact, source MAT024 behavior or a new rendered result.
The prior standalone T3 stage and shared helper extraction remain the numerical
baselines. The [binding producer](../mixed_binding/README.md) and
[resident design](../../../../../planning/T3_RESIDENT_BATCH_DESIGN.md) define
the immutable union and subsequent numerical gates.

## Production boundaries

The two existing batches add `InitializeJoined(config,binding)`. The producer
supplies each actual typed reference/connectivity; no user-owned partial mass
or altered derived reference is accepted. Each family retains its own native
history/cache and obtains the **complete** union reference positions and native
mass/J for actual owner validation. An individual family may leave nodes
uncovered; its binding cannot. Rest/zero velocity/unit orientation/free masks
and reciprocal mass/J checks retain the standalone operation and `1e-12`
reciprocal-product tolerance. Full-width source IDs and represented positions
were checked by the immutable binding producer.

Joined mode is private, immutable and initially `PrescribedFields` only. Each
family sets `kinetic_available=false` and its four kinetic fields to zero;
those zeros mean unavailable, not measured zero energy. It still reports its
own native EINT2 and internal RHS work; QEPH additionally retains its own EVIS.
The T3 type acquires no fictitious hourglass field. Standalone initialization,
kinetic arithmetic and usage policies retain their previous behavior.

`ShellBatchPublication` is a closed two-family coordinator, not an element
registry or second dynamics owner. It borrows the two batches, which must
outlive it, and claims exactly one coordinator scope in each participant.
Duplicate attachment rejects before allocation. Its only device allocation
holds the immutable union mass ledger and scalar kinetic scratch; there is no
per-step allocation or copied nodal/material state. The reusable
`ShellBatchJoinedModel.h` copies native union values into both existing models.
Formulation force/history arithmetic stays in the already qualified ports.

1. Initialize the sole owner and both joined participants; assemble/discard
   both actual initial rest caches before `Initialize(owner,qeph,t3)`. The
   coordinator checks the retained first successful assembly source identities
   against the actual owner's immutable mass/masks and accepted kinematic
   buffers before device allocation or zero-diagnostic publication. Later raw
   assembly cannot replace these first-binding records.
2. Advance one prescribed owner trial through the existing receipt-gated
   staggered operation. Evaluate both native material candidates from that
   same endpoint, in either order, then stage the typed result readbacks.
3. `Prepare` checks both pending results, full inventory, immutable component
   configuration/qualification/usage, exact accepted stamp/stream and actual
   owner token/buffer tuple **before** launching kinetic measurement. It
   measures base and endpoint translation, native total rotation and separate
   physical/area-added isotropic rotation exactly once over the union. Their
   timestamps remain the owner's collocated-start/previous-midpoint times.
   At epoch zero, the retained initial source identities are also checked
   again; this host-only comparison does not consume a pending CUDA error.
4. The caller finishes independent work/geometry/native checks and any output
   preparation before `Commit`. Commit checks the complete measured result,
   both pending contributors, actual token/buffers, scope and receipt again.
   It performs one `CompleteNodalValidation` and one owner commit.
5. After owner success, only the shared `Impl::Publish` slab swaps, scalar
   stamp/diagnostic assignments and scratch invalidation remain. No CUDA call,
   allocation, readback, numerical check or callback follows that boundary.

Every `Prepare`/`Commit` failure discards the nodal trial and
both material trials. Standalone commit functions reject joined participants
before any owner commit. An individual typed evaluation failure still requires
the coordinator/caller to close the whole attempt; force assembly scratch is
disposable and is not promised a whole-batch rollback. Accepted material/cache,
clock and host output survive numerical failure. CUDA failures poison the
joined scope; only unchanged accepted metadata and untouched caller output
are guaranteed, not readable-device recovery. A pending CUDA error at commit
is deliberately left for the owner to observe before publication.

The accepted common kinetic cache is output metadata attached to the exact
owner stamp. It does not interpolate velocity, advance time or evolve an
independent energy state. Native total J is never reconstructed from partitions.
Neither native donor runtime is linked into production.

## Frozen fixture, independent checks and failures

The five-node edge fixture, areas, source IDs and materials are exactly those
in the binding README: Q4 `{0,1,2,3}`, scalene T3 `{1,4,2}`, rho=1024 kg/m³,
t=1/32 m, E=2e6 Pa, nu=.3. Both components use configuration
`0x4d42314d4f444c31` and the new prescribed qualification
`0x4d42315052455331`. Fixed h=1/1024 s; the first kick is h/2, subsequent
kicks and all physical drifts follow the retained owner timing.

At each native reference coordinate (x,y), prescribed velocity is
`(.001*(x+.25*y), .0005*(y-.5*x), .00075*(x-y))` m/s and world spin is
`(.002*y, .003*x, .001*(x+y))` rad/s, multiplied successively by
`{1,0,-1,0}`. External state-independent loads are frozen from these targets,
previous targets and native union total m/J. Internal caches are not consumed
in any committed interval. Both zero targets are actual rate holds. Independent
closed positions/velocities and commuting-axis sine/cosine quaternions check
the resulting schedule; no second integration implementation is used.

Eight actual CUDA test functions are proposed:

* Startup checks complete union mass, scalar inertia partitions, typed zero
  histories, unavailable family kinetics and one nodal owner.
* Four prescribed load/hold/reverse/hold intervals compare both native force
  chains and histories, independent source work and the single kinetic ledger;
  allocation bytes/counts remain unchanged.
* A nonzero accepted QEPH-then-T3 cache scatter is compared with complete
  native CUPDTN3/C3UPDT3 and mapped to the five physical nodes, then discarded.
  The original native T3 bridge remains inside its four-node capacity.
* Invalid joined counts/policies/capacity, owner mass/J/rest/free-mask faults,
  and geometry-identical different source inventories/configurations reject.
  Each family independently checks a bad node absent from its own cell.
  Foreign initial mass or kinematic buffers with plausible metadata fail
  coordinator authentication for either family, including a numerically valid
  foreign mass that conceals the actual owner's incorrect mass. Reassembly
  cannot replace the retained source record; fresh valid participants retry.
* Missing contributor, wrong owner/stream, plausible metadata with foreign
  buffers, and null/overlapping precommit output preserve all accepted results.
* Duplicate scope, standalone commits, stale/tampered global or typed data,
  invalid receipts and a discarded required participant cannot publish.
* A T3-only collapse after successful QEPH evaluation and a Q4-only collapse
  after successful T3 evaluation both preserve nonzero accepted state/history.
  Subsequent clean retry matches a separate clean owner and both native chains.
* A real invalid CUDA launch immediately before joint commit prevents owner
  and both material publications and poisons the scope. It uses the retained
  clean-error precondition and permits the known invalid-value/configuration
  runtime variants; it does not claim device readback recovery.

There are 10 successful/native-checked pair intervals, or **20 native cell
interval checks**: five per family in the state/scatter tests and five per
family in the rejected-path clean retry tests. Rejected candidate evaluations
are additional tests, not successful native/trajectory evidence.

All numerical budgets are frozen before execution. Native field comparisons
reuse the retained `2e-12` dimensional force-port contracts and independent
material/power floors, including `2e-22` J where applicable. The global/source
work ledger uses `256*epsilon*sum_absolute_terms + 1e-12*1e-6 J`; the nonzero
experiment scale is exactly `1e-6` J. The independent long-double world-edge
area/atan2 mass oracle is reused. The owner target tolerance remains `2e-13`
with its existing component scales. Before the final receipt, **each family**
must separately exhibit membrane work >1e-12 J, bending work >1e-14 J and force
>1e-5 N. Exact native scatter order, identity, cached results and failed-output
checks have no numerical tolerance. No floor may be enlarged after a failure.

## Registration and bounded execution

Root owns shared CMake/Bazel wiring, live source maps and all actual builds/GPU
runs. The new production library consists of `ShellBatchPublication.cpp` and
`ShellBatchPublicationKernels.cu`, with the public/private headers alongside.
It depends on the existing typed batches, binding and nodal owner. Each typed
batch gains only the binding/`ShellBatchJoinedModel.h` dependency; the binding
depends on startup headers, so no publication/batch dependency cycle is added.

The `mixed_shell_batch_check` target needs `MixedShellFixture.cu`,
`MixedShellLedger.cu`, `MixedShellStateTest.cu` and `MixedShellFailureTest.cu`;
the existing `qeph_q1_native`, `t3_r3_native`, GTest and CUDA runtime supply
only test dependencies. Reuse C++/CUDA17, strict FP64/no-FMA/precise divide and
sqrt/no-FTZ flags and a single serialized GPU test. No driver or standalone
alternate qualification project is introduced.

There are six existing owner allocations plus one per typed batch and one
kinetic scratch allocation: **nine explicit allocations**. Both typed batches
retain their 1 MiB ceilings; the kinetic scratch has a compile-time 1 KiB cap.
The retained host ABI probe measured **14,832 B QEPH, 6,224 B T3 and 304 B
publication storage**, each alignment 8: **21,360 B** across those three
device allocations, plus the existing owner's six allocations. The evidence
is `crash-work/checkpoints/robo-dyna-restart-20260910T0010Z/abi/measurements.json`
and `crash-work/reports/mixed-shell-abi-build-1.json`. The new retained source
identity records are host-only and do not alter those device storage types.
Actual CUDA compilation/layout and allocation measurements are **pending**;
the prior standalone T3 6,216-byte record is not a current allocation claim.
Freeze the source map after shared registration and identity changes settle,
before the first numerical execution. Old standalone QEPH/T3 batch, native force,
owner timing and scoped QEPH response/contact regressions remain required.

`prepare_source_map.py` lists the explicit linked/build inputs and recursive
local C/C++ includes without hashes by default. After source review and shared
registration settle, `--write` creates `source-map.json` once; `--check` compares
the complete map and current input closure. The original/extracted native
Fortran closure is delegated to the two pinned native manifests and owning
verifiers. Compiler, CUDA, Fortran runtime, GTest and system headers remain
explicit external dependencies; this map is not a hermetic toolchain image.
