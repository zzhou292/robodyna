# Fixed self-contact runtime transaction

`SelfContactTransaction` privately owns the complete fixed-facet source path:
one `SelfContactBroadphase`, accepted and candidate
`FixedTriangleFeatureDiscovery` instances, current regularity, represented
interval crossing, one `SelfContactForceAssembly`, and exactly one
`ShellPhysicalScratchParticipation`. Startup reauthenticates the retained
active-use physical binding, actual publication owner/participants, startup,
configuration and qualification IDs, and the owner's nondefault stream. It
preallocates both device owners plus host device-key readback, the complete
surface-parent map, parent/facet offsets, descriptors, triangles, paths,
accepted/candidate facet pairs, exact events/certificates, policy outcomes,
snapshots, and activity. The caller can obtain only the immutable roster entry
and a borrowed post-seal policy publication.

Accepted assembly copies the actual owner snapshot, certifies every selected
parent, runs the complete current broadphase, validates its full device pair
readback, expands every S0 pair to every facet pair, discovers all fixed
features, constructs each admitted directed VF event from the exact discovery
weights and active-use classifier, and assigns canonical source order. Only
then does it run the existing deterministic VF force/STI implementation and
record the same owner/token/view on the private issuer. There is no public
event, triangle, pair, discovery, crossing, or source-order input.
Every later failure discards owner, common publication, force and issuer trial
scratch. Caller output changes only after both operations succeed.

Candidate sealing copies actual
accepted and prepared positions from `FENodalState` into fixed startup storage,
runs a complete swept broadphase over the authenticated linear endpoints,
expands every returned pair, runs current regularity over every active parent,
rebuilds every exact fixed facet, and reruns complete discovery and represented
interval crossing. Unresolved results fail. The fixed policy rejects nonlocal
intersections and EE crossings and admits a VF crossing only through the full
transaction-owned accepted event certificate (feature, maps/weights, area,
classification, and canonical source order). Local fixed-facet exclusions
remain explicit. Exactly static, nondegenerate paths use exact triangle
intersection directly, so disjoint triangles whose AABBs merely touch do not
exhaust subdivision work. Empty broadphase, discovery, event, crossing, and
policy publications are valid and still produce mandatory participation.

The final nonaggregate `SelfContactTransactionReceipt` is tied to the exact
owner/base/attempt/source/configuration/qualification/active-use identities.
Only that receipt can expose the private typed physical receipt in a
`ShellPhysicalScratchReceiptRoster`.

This slice intentionally retains a private all-one activity seam and rejects
any internally unsupported removal state. It accepts no public activity
pointers. Integration with the parallel activity-authority work must replace
only that private seam with the physical-publication-authenticated,
active-use-ordered base/current receipt. Integration with the parallel complete
assembly-view authentication must replace the accepted source-only check used
before broadphase; the force owner remains the final full-view consumer.
Rigid-arc/nonlinear paths are explicit candidate failures rather than being
misrepresented as endpoint chords.

## Author host gate

Run with one CPU and a 512 MiB cap under
`crash-work/reports/connection-author.lock`:

```sh
cmake -S lib_utest/qualification/self_contact_transaction \
  -B <host-build> -DCMAKE_BUILD_TYPE=Release
cmake --build <host-build> --parallel 1
ctest --test-dir <host-build> --output-on-failure
```

This gate runs value, exact-cap, crossing-policy, source and C++ syntax tests.
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
and deterministic retry. CUDA execution is intentionally left to the parent.
