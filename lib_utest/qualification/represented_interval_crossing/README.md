# Represented interval crossing certificate

`RepresentedIntervalCrossing` owns bounded host staging and immutable published
results for fixed-triangle pairs. `LinearNodalV1` means that each facet vertex
follows the exact real affine path between its two represented binary64
endpoints over normalized time `[0,1]`. `RigidArc` and `Nonlinear` are explicit
unsupported declarations and publish `Unresolved/UnsupportedMotion` without
examining endpoint-union boxes.

The implementation iteratively partitions normalized time at exact dyadic
values. A preallocated depth-first cell stack has exactly `max_depth+1` rows;
one reusable exact scratch packet serves every visited cell, so depth 52 does
not retain exact values on the process call stack. At a sampled dyadic time it
converts every endpoint to an exact dyadic
integer/exponent value and applies exact triangle SAT, including transverse and
coplanar axes. An intersection is therefore an existence certificate at the
reported exact numerator and depth. A deterministic immutable VF or EE feature
key is selected where one exists; a complete triangle-intersection key remains
for transverse edge/face piercing.

The existing exact common-translation branch compares exact dyadic endpoint
displacements for all six vertices. When equal, every relative geometric
predicate is constant throughout the represented interval. A crossing from
this branch now carries `ExactCommonTranslationTransverse` or
`ExactCommonTranslationCoplanar`. `HasExactCommonTranslationProof` reads that
provenance; `BaseIntersectionGeometry` preserves the original witness shape.
Classification, source feature, earliest witness and one-unit work charge are
unchanged. Separated and unresolved results retain their previous geometry.
The two tags append to the byte-sized enum; no result fields, forecast sizes
or frozen fixture layouts change. Historic fixture enum ordinals remain valid.

The tag proves geometric invariance, not local source topology, force ownership
or transaction admission. Static nonlocal overlap receives the same invariance
provenance and remains a crossing. A consumer needs its own authenticated
topology and physical transaction obligations. This slice does not change the
candidate controller or the standalone continuous-local thickness contract.
Rounded-equal displacements are insufficient; declared curved/nonlinear paths
remain unsupported even if their endpoint displacements match.

Separation has a different proof. On each accepted time cell, exact quadratic
Bernstein controls certify both triangle normals nonzero. Because every admitted
vertex coordinate is linear, its extrema on that cell occur at the two
endpoints. A strict disjoint coordinate interval therefore certifies separation
for that cell; both child cells must certify before their parent does. This
linear-path endpoint enclosure is never used as evidence of crossing. If exact
contact is not sampled and separation cannot be proved within the declared
depth/work, the result is `Unresolved/WorkExhausted`.

Exact sampled degeneracy is `Unresolved/DegenerateGeometry`; an unsampled
regularity uncertainty also cannot become a separation certificate. The exact
predicate integer has a fixed 16,384-bit stack representation, enough for all
binary64 exponent alignment and predicate products used here. A checked range
failure is explicit `Unresolved/ExactArithmeticRange`.

Pair/path inputs are validated completely, canonicalized by immutable source
path and feature keys, sorted, and duplicate pairs removed before evaluation.
Every vertex source instance must match its path. Source-vertex, reduced dyadic
source-edge and parent-interior key fields follow `FixedContactFacetBinding`;
edge endpoints must be sorted, and boundary edges use EID zero while interior
edges use the path parent EID. A preallocated sorted `3*max_paths` ledger admits
shared keys only when endpoint bits and declared motion agree. Compatible
duplicate path/edge identities are admitted independent of winding; conflicting
duplicates reject.

Per-pair work exhaustion is a complete unresolved record. Checked total-work,
result, path, pair, or host-byte exhaustion fails the whole call and preserves
the previous complete publication. A smaller valid query can then retry.
Forecast bytes itemize path indices, canonical pairs, two result buffers, the
vertex ledger, DFS cells and exact scratch in addition to the implementation.
Every input is disjoint from all six owned arrays, exact scratch and control.
`results()` expires on the next successful query, move or destruction; every
failed query preserves its prior address, count, completeness and bytes.

This slice owns no broadphase, active parent-use policy, source selection,
thickness/area/stiffness, force, rigid/CIN response, timestep choice, adaptive
controller, CUDA operation, or physical-owner transaction.

## Prospective lexical path-roster reuse

This isolated change is authored for review; it has not been built, tested or
timed. The active vehicle qualification continues on its unchanged binary.

The internal generic `represented_interval_crossing::BatchAccess::Certify`
owns one synchronous canonical batch traversal. Its native busy lease spans
all slices. A lexical `PathRoster` authenticates the full original immutable
path/vertex roster once before pair work; it is neither retained in the owner
nor returned to the caller. No callback or caller-supplied validity flag crosses
this API. Each slice keeps its original phase, pointer/range, pair/result and
work admission, worker execution, canonical fold and publication swap.
Public `Certify` constructs a fresh roster and retains its existing contract.

The transaction adapter only delegates to this native entry. Its full
canonical input checks keep their original priority. Private output scratch
must be disjoint from inputs, the native owner object and every retained
native region, including expired publication/staging capacity. A later failed
slice preserves the last successful native slice; it returns no complete outer
view. No extra retained arrays, worker threads, device memory, source owners,
clock, numerical predicates or limits are introduced.

`BatchReport::path_roster_work` records bounded stack-only authentication,
path/vertex-row and sort counts. These are not certificate work, physical
diagnostics or fixture fields. They support an operation-count regression;
they make no timing or speedup claim. `BatchRosterTest` compares a separate
legacy loop over public `Certify` with the compound operation field by field,
including late failures, empty input, unused malformed paths, publication
retention, retry reauthentication and expired-view rejection. Existing owning
native tests, transaction batch/force/rollback tests, frozen replays and a
guarded production continuation remain required before integration.

## Author host gate

From a fresh build directory, with one CPU and a 512 MiB guard:

```sh
cmake -S lib_utest/qualification/represented_interval_crossing \
  -B /tmp/represented-interval-crossing -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/represented-interval-crossing --parallel 1
ctest --test-dir /tmp/represented-interval-crossing --output-on-failure
python3 -B \
  lib_utest/qualification/represented_interval_crossing/verify_sources.py
```

Bazel wiring is
`//lib_utest/qualification/represented_interval_crossing:{host_check,source_identity}`;
the source proof declares complete runfiles. Both remain root-only execution
gates. Root must later compose this API with the
fixed-feature discovery output and run the actual selected initial/short
interval inventory. CUDA/device implementation, full-source counts,
rigid/nonlinear path enclosures, force/STI and common publication remain
separate gates.
