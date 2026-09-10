# Active shell resident storage qualification

The production QEPH/T3 participants and common publication scratch now allocate
active-count arenas at initialization. Each family keeps one elastic allocation
and optionally one plastic allocation. Native references, both force/history
slabs, per-parent status and native node M/J arrays share the elastic arena. The
plastic arena holds the active source curve pool, parent parameters and both
section history slabs. The family's existing accepted slab pointer remains the
sole selector for shell and plastic state. No step allocates or resizes storage.

`lib_utils/BoundedArena.h` owns only checked layout arithmetic and fresh host
staging. It has no CUDA call, accepted index, state, stream or solver. Small typed
layout adapters construct host headers and then rebase their fields into the
single device allocation. Host readback retains those device addresses in a
header shadow; it never reads a pointer field through a device-resident header.

Existing callers retain the 128-parent/128-node defaults. Larger batches must
set `config.storage_limits.max_parents/max_nodes` (hard ceilings 1024/2048) and
provide sufficient `config.max_device_bytes` (default 1 MiB, hard ceiling 8 MiB).
The independent host payload cap defaults to 8 MiB and is bounded by 32 MiB. Its
conservative startup ledger includes resident-header copies, initialization and
readback arrays, identity/reference staging, and retained immutable geometry and
material catalog payload. Allocator bookkeeping and CUDA runtime memory are not
module-owned payload. Combined elastic/plastic byte admission precedes borrowed
reference or curve reads. Arithmetic uses division and subtraction before size
multiplication/addition.

`ShellPublicationLimits` is an optional fourth initialization argument. Its
legacy node limit is 128; an assembly explicitly opts into up to 2048. Common
native M/J arrays are active-count and moving-startup readback is 13 doubles per
active node. Its default device/host caps are 128 KiB/1 MiB. The original serial
native mass/kinetic reduction and 64-worker strided candidate arithmetic are
unchanged. The common commit still advances one nodal owner, then infallibly
publishes both family slabs; plastic state adds no second selector or clock.

The CMake owner `active_shell_collection_check` is synthetic capacity evidence:
804 QEPH parents, 111 T3 parents, 1030 nodes, six materials/sections, two curves
with 63 total points, and high T3 source node IDs. It reserves isolated final
parents to inject late failures, verifies every history advances, checks tail
forces/material results against the already-qualified direct host value
operations, and checks elastic/plastic readback, rejection, exact retry and
stable allocation counts. Those value operations are a storage/parameter-offset
oracle, not a new independent constitutive oracle. Existing native suites retain
that responsibility. This package is not source Yaris geometry, contact,
constraint recurrence, long-trajectory admission, or a rendered crash.

The host target `active_shell_layout_check` covers alignment, overflow, atomic
failed layout updates, active footprint growth, rebasing and maximum admitted
sizes. Existing smaller collection/plasticity tests retain their physics checks
and now compare the exact active allocation footprints.

Standalone guarded qualification can configure this directory with
`CMAKE_CUDA_ARCHITECTURES=120`, then build the two targets and run their CTests.
The existing mixed T3 qualification also includes both targets. GPU discovery is
required; no successful skip exists. Use the workstation's serialized resource
guard and do not run simultaneously with another CUDA/native build or GPU job.
Implementation authors did not execute compilation or tests; qualification is a
separate handoff gate and must be recorded by the composing build owner.

The standalone build and all six tests pass on the RTX5090 (2026-09-10):
`crash-work/reports/active-shell-storage-configure-1`,
`active-shell-storage-build-{1,2,3}`, and `active-shell-storage-tests-3`.
The first two retained test attempts exposed missing fixture setup: explicit
large host geometry admission and the live-owner overload required to bind
uniform initial translation. Those fixture calls were corrected; production
arithmetic/storage did not change. Existing legacy suite qualification follows
separately.

The integrated original-part harness passes all 39 selected groups after the
active storage change (`active-shell-legacy-tests-1`). The owning Bazel arena,
QEPH/T3 and common publication targets build in
`active-storage-owner-owning-bazel-1`. Larger grouped dynamics remains a
separate qualification.

## Multi-block candidate evaluation (2026-09-10)

TL34dadf3 evaluates independent QEPH/T3 parents in64-thread blocks, followed by a
single same-stream finalizer that retains the original first-failure scan and
ordered diagnostics. It adds no state, arena or per-step allocation and changes
no element, material or force-scatter arithmetic. All7 focused host/CUDA
functions pass, including simultaneous63/64/final-parent failures for elastic
and plastic paths, complete accepted preservation and exact retry. Owning Q/T
Bazel batch targets build. Independent review found no blocker.

On the actual804-QEPH/111-T3 Yaris assembly, the128-step and1024-step trajectories
have byte-identical accepted meshes, OBJ files, plastic fields and complete
interval/index ledgers relative to their retained baselines. Reports:
`crash-work/reports/shell-candidate-grid-tests-1.json`,
`source-assembly-candidate-grid-parity-1.json` and
`source-assembly-candidate-grid-parity-2.json`. The yielding1024-step run takes
115.170s versus116.835s, and the128-step run14.354s versus14.561s. These single-run
differences do not establish a material speedup. Optional component-stage timing
is the next measurement; CUDA API wall time alone does not identify kernel cost.
