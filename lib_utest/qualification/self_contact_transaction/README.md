# Fixed self-contact runtime transaction

`SelfContactTransaction` privately owns the complete fixed-facet source path:
one `SelfContactBroadphase`, accepted and candidate
`FixedTriangleFeatureDiscovery` instances, current regularity, represented
interval crossing, one `SelfContactForceAssembly`, exactly one
`ShellPhysicalScratchParticipation`, and one
`SelfContactPhysicalActivity`. Startup reauthenticates the retained
active-use physical binding, actual publication owner/participants, startup,
configuration and qualification IDs, and the owner's nondefault stream. It
preallocates both device owners plus one complete host device-key readback,
the complete surface-parent map, parent/facet offsets, descriptors, accepted
and prepared triangles, canonical parent-pair cursors/heap, fixed facet-pair
and exact-geometry chunks, a global accepted-event certificate/hash ledger,
optional detailed policy outcomes, and snapshots. It does not allocate the
complete facet-pair, feature-task, intersection, crossing, or policy arrays.
The activity authority separately owns one exact
accepted/current and complete-family readback arena. The caller can obtain only the immutable roster entry
and a borrowed post-seal policy publication.

Accepted assembly first captures activity from the actual QEPH/T3/QBAT
participant assembly. It copies the actual owner snapshot, certifies every selected
parent, runs the complete current broadphase, validates its full device pair
readback and validates its strict device order. A fixed cursor heap merges
every active parent Cartesian expansion into immutable facet-key order and
materializes one bounded chunk at a time. Every chunk executes all six VF and
nine EE tasks per facet pair. EE coverage remains local to the same exact
facet pair; nonlocal intersections reject. Admitted VF certificates merge
through a fixed-capacity global hash ledger, where repeated keys must agree.
The complete unique event count is known before the force cap is checked, then
certificates are sorted and assigned global canonical source order. Only
then does it run the existing deterministic VF force/STI implementation and
record the same owner/token/view on the private issuer. There is no public
event, triangle, pair, discovery, crossing, or source-order input.
Every later failure discards owner, common publication, force and issuer trial
scratch. Caller output changes only after both operations succeed.

Candidate sealing requires the exact `ShellPhysicalDiagnostics` returned by
`PreparePhysical`, captures publication-authenticated prepared activity, and copies actual
accepted and prepared positions from `FENodalState` into fixed startup storage,
runs a complete swept broadphase over the authenticated linear endpoints,
filters inactive pairs and runs current regularity over active, removing, and
long-inactive parents,
rebuilds every exact fixed facet, and reruns discovery and represented
interval crossing on the same canonical chunks. Global canonical vertex/edge
coordinate ledgers are checked before chunking. Unresolved results fail. The fixed policy rejects nonlocal
intersections and EE crossings and admits a VF crossing only through the full
transaction-owned accepted event certificate (feature, maps/weights, area,
classification, and canonical source order). Local fixed-facet exclusions
remain explicit. Per-pair outcomes can be retained in full when the caller
reserves the exact census; otherwise no partial outcome view is published and
the complete canonical counts/digest are folded into `policy_summary()`.
Exactly static, nondegenerate paths use exact triangle
intersection directly, so disjoint triangles whose AABBs merely touch do not
exhaust subdivision work. Empty broadphase, discovery, event, crossing, and
policy publications are valid and still produce mandatory participation.

Exact closest-stratum predicates remain the feature identity authority.
If an ordinary binary64 face projection rounds onto its exact boundary,
discovery publishes deterministic strictly interior represented weights and
an explicit upper reconstruction error instead of calling the rounded point
exact. An edge parameter for which no two positive binary64 endpoint weights
exist remains an actionable `EdgeInteriorRepresentation` failure. Discovery
reports the exact pair, directed task, and arithmetic reason.

An unforced EE still contributes no line area and no force. A penetrating EE
may share an admitted VF policy only on the same exact fixed-facet pair;
parent-pair-only coverage is rejected because a remote VF cannot prove force
coverage for an unrelated EE elsewhere on the same parents.

The final nonaggregate `SelfContactTransactionReceipt` is tied to the exact
owner/base/attempt/source/configuration/qualification/active-use identities.
Only that receipt can expose the private typed physical receipt in a
`ShellPhysicalScratchReceiptRoster`.

No transaction-owned all-one activity array remains. Accepted and prepared
activity receipts stay private inside transaction receipts, become stale on
discard/publication/generation change, permit `1->0` removal and `0->0`
long inactivity, and reject `0->1` reactivation. Removing and long-inactive
parents remain represented in regularity but own no candidate facet pairs,
events, or policy outcomes.
Rigid-arc/nonlinear paths are explicit candidate failures rather than being
misrepresented as endpoint chords.

## Vehicle-scale limits and memory

`SelfContactTransactionLimits::Vehicle` takes caller-supplied exact census
counts and explicit chunk/event/policy/work/byte caps. It derives all component
count limits with checked arithmetic; it contains no Yaris constants. Forecast
reports complete parent/facet caps separately from allocated chunk capacities,
plus exact readback, cursor, heap, arena, component, startup, and device bytes.

For the measured V5 shape (376,930 owner nodes, 337,092 selected parents,
653,055 facets, 1,584,464 parent pairs and 5,989,248 facet pairs), a 4,096-pair
chunk and folded policy use exactly 1,401,633,232 transaction-arena bytes with
one event slot. One million force/event/certificate slots use 2,913,631,720
arena bytes. The fixed subranges include 12,675,712 parent-key bytes,
44,364,992 cursor bytes, 6,337,856 heap bytes, and 32,768 facet-pair chunk
bytes. The actual event ledger capacity must come from a complete streamed
census; the earlier broadphase receipt explicitly did not run feature/force
admission.

Asymptotic storage is `O(nodes + facets + parent_pairs + event_cap +
chunk*(feature/work caps))`, not `O(facet_pairs)`. Traversal is
`O(facet_pairs log parent_pairs)` before exact geometry. The expected
full-V5 performance blocker is CPU exact work: 89,838,720 VF/EE tasks plus up
to 5,989,248 multiprecision interval certificates. Chunking makes that work
representable and failure-atomic; it does not make it fast.

## Caller migration

Remove `config.activity_policy`; the physical publication is now the only
activity authority. Pass the exact common diagnostics returned by
`PreparePhysical` before the prepared view:

```cpp
transaction.SealCandidate(
    owner, token, common_diagnostics, prepared, accepted, &receipt);
```

Tune activity capacity through `limits.activity`. Forecast activity storage is
`forecast.activity`; the old `forecast.parent_activity_bytes` field is gone.
Allocation reporting is split into `allocations().activity` for the one host
activity arena and `allocations().device` for transaction device ownership.
CMake and Bazel users need no new direct dependency when they already consume
`tl_self_contact_transaction` or `//lib_src/collision:self_contact_transaction`.

## Author host gate

Run with one CPU and a 512 MiB cap under
`crash-work/reports/connection-author.lock`:

```sh
cmake -S lib_utest/qualification/self_contact_transaction \
  -B <host-build> -DCMAKE_BUILD_TYPE=Release
cmake --build <host-build> --parallel 1
ctest --test-dir <host-build> --output-on-failure
```

This gate runs value, exact-cap, crossing-policy, million-parent-pair
streaming, chunk-boundary event/identity/fold equivalence, source and C++
syntax tests.
It invokes no NVCC or GPU.

## Root CUDA gate

The parent CUDA workstation owns:

```sh
cmake -S lib_utest/qualification/self_contact_transaction \
  -B <cuda-build> -DCMAKE_BUILD_TYPE=Release \
  -DSELF_CONTACT_TRANSACTION_CUDA=ON \
  -DCMAKE_CUDA_ARCHITECTURES=<root-arch>
cmake --build <cuda-build> --parallel 1
ctest --test-dir <cuda-build> --output-on-failure
```

That target uses real `FENodalState`, physical publication, current regularity,
fixed discovery, represented crossing, VF force and CIN STI. It covers missing
mandatory receipt rollback/retry, initial half-kick, ordinary interval and
allocation stability, plus an exact pass-through unresolved-reason rollback
and deterministic retry. It also covers actual T3 removal, long-inactive
participation, stale accepted authority, forged physical diagnostics, and
exact rollback/retry. Dedicated accepted-stage rigid gates retain genuine execution-owned
merged PART and plain declarations. One proves a discovered same-body VF is
removed before event/force/STI publication; another proves a transaction-owned
VF produces equal/opposite nodal loads and that the actual owner uses each
body's merged resultant/moment before its dense anisotropic inverse. The latter
also carries a deliberate PART/plain numeric-ID collision and distinguishes
the merged response from an endpoint-wise inverse sum. They discard rather
than claim interval acceptance because candidate sealing deliberately reports
`UnsupportedMotion` while rigid arcs remain outside `LinearNodalV1`; nonlinear
rigid interval admission remains a full-V5 dependency. The pass-through is driven by a token-authenticated
assembled nodal force, not an inconsistent initial nodal velocity. CUDA
execution is intentionally left to the parent.
