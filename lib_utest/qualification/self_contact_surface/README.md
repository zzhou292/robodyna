# Immutable shell self-contact source, S0

This host-only increment binds an explicit shell subset to one prepared
`ShellPhysicalBinding`. It implements no collision query, activity observation,
force, response, timestep, owner/participant, or original contact-card adapter.
The named profile is
`FrictionlessReferenceThicknessShellSubsetV1`: **current physical Q4/T3
midsurfaces plus constant reference half-thickness distance metadata**. It does
not establish parity with the original 861-PID automatic single-surface card.

`SelfContactParentSelection` supplies catalog row, family/index, EID and PID.
Every field must match the retained catalog and native source reference. The
selection is copied; all physical backing remains lifetime-retained. No source
card parsing or reference/material equation is introduced. QEPH/T3 top/bottom
reference placements and QBAT offsets are explicitly rejected. Centered rigid
skins retain their geometry, thickness and zero-point role.

Parent records retain input order and native cyclic node order. `faces()` sorts
parent indices by source EID. Vertices sort by source NID; edges sort by their
unordered source-NID pair; incident uses sort by EID/local slot. Distinct source
faces always survive, including coincident layers. Equal coordinates with
different source NIDs never merge. A shared feature can have parents with
different thickness, role, material and later activity: all uses and per-parent
half-thickness are retained, and no canonical feature receives a guessed radius.

`VertexInFace` and `EdgesShareVertex` answer **topological feature incidence**
only, including an explicit Invalid result. Neither marks entire adjacent face
pairs safe to ignore. There is no same-PID/body/CIN/tie exclusion. Future runtime
must apply its reviewed active-use geometry/support policy.

The public Q4/T3 records reuse existing zero-offset surface maps, so the separate
half-thickness does not silently move their reference planes or supply a normal
Jacobian. Reference positions are retained through the existing physical source;
they cannot stand in for a current stamped owner view.

## Resource and failure contract

`Preflight` returns values, avoiding caller destination aliases. It checks count,
profile, pointer-range arithmetic, complete source identity, reference placement
and duplicate parents. Count/byte bounds precede borrowed reads or allocation.
The temporary existing `SourceIdentityIndex` detects the earliest repeated
selection after the ordered source validation phase; it retires before arena
allocation. Feature cardinality is checked during the staged build, so a lower
unique vertex/edge cap can still reject `Initialize` after a successful source
preflight. Failure leaves an empty handle reusable; success is immutable.

One arena has exact forecast capacities: selected parents, at most four corner
uses per parent, bounded unique vertex/edge prefixes, and source-EID face order.
Inactive capacity tails are still charged. `arena_bytes` is the exact arena
allocation; `owned_payload_bytes` additionally charges the complete retained
physical payload, handle/implementation and 64-byte shared control reserve.
`startup_payload_bytes` conservatively adds the validation index even though
its allocation retires before the arena. These are payload limits, not RSS or
device forecasts; this slice allocates no device storage. All arithmetic uses
the existing checked `BoundedArenaLayout`; hard count caps also prove uint32
feature indices and bounded sorting depth. Standard allocator bookkeeping and
small stack values are outside the payload convention.

`OutputDisjoint` excludes the handle, implementation, complete arena (including
unused tails), and existing complete physical/catalog/ledger/rigid ranges. The
same source-range predicate guards initialization's destination; input selection
overlap with the destination is rejected before any selection read. Read-only
views confer no mutable source or dynamics permission.

## Host gate

Seven host functions cover three-family source/native thickness/ordered mapping,
all 0/1/3/4-point roles, shared unequal-thickness layers, separate coincident NIDs,
canonical feature identity under selection reordering, remote-edge incidence,
late typed identity failures, duplicate selections, unsupported reference planes,
inclusive byte caps, late feature caps/retry, overflow before borrowed reads,
exact retained source identity/lifetime, and source/output aliases.

From the workspace root (author: 1 CPU, 512 MiB; no native/NVCC/CUDA):

```sh
python3 Total-Lagrangian-FEA/tools/run_bounded.py \
  --report crash-work/reports/self-contact-surface-author.json \
  --lock crash-work/reports/self-contact-surface-author.lock \
  --cpus 1 --min-available-gib 1 --max-rss-gib .5 --timeout 360 -- \
  bash -c 'cmake -S crash-work/worktrees/self-contact-source/lib_utest/qualification/self_contact_surface -B crash-work/build/self-contact-surface-author -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS_RELEASE="-O0 -DNDEBUG -g0" && cmake --build crash-work/build/self-contact-surface-author --parallel 1 && ctest --test-dir crash-work/build/self-contact-surface-author --output-on-failure'
```

Root owning target: `//lib_utest/qualification/self_contact_surface:host_check`.
The C++ CMake chain locates installed CUDA headers for the existing physical
output-range declarations, but compiles no CUDA source and creates no context.
Existing production edits are limited to an additive collision build target;
two test-fixture visibility lists also admit this qualifier. No historical
source receipt is refreshed.

## Root full-source next recipe (not executed by the author)

1. Reuse the existing V5 original `VehicleShellExecution::physical()` and its
   complete catalog; do not parse source files again. Enumerate catalog rows into
   exact typed selections. Record source/domain/physical profile identity and
   the selected PID/EID inventory under an explicit shell-subset policy.
2. First run preflight on the intended inventory. Unsupported top/bottom/offset
   rows must be counted and reported. An explicitly named centered-only subset
   may select the remaining rows; do not silently drop rows or claim all 861
   source PIDs. Original source-card set selection remains a separate adapter.
3. Use `SelfContactSurfaceLimits::Vehicle()`, log every forecast field and source
   count before construction. Preserve the normal root host guard. Construct,
   compare every selected face's family/EID/PID/ordered global source NIDs,
   thickness bits and 0/1/3/4-point role with the retained source. Report actual
   unique vertices/edges/use counts and exact reservation, never guessed counts.
4. Exercise an exact startup cap, one byte short, final-row identity failure and
   retry; keep a binding copy alive after source wrapper destruction and query
   its retained physical identity. This is a **source binding gate only**.

No self-contact force, initial-overlap resolution, area/radius ownership,
continuous geometry query, exclusion authority, mass majorant, or folded-vehicle
trajectory admission follows from S0 passing.
