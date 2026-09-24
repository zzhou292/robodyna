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
predicate integer has a fixed 16,384-bit inline representation, enough for all
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

## Lazy exact normal reuse

Each worker's existing `ExactScratch` retains six additional exact normals, one
for each triangle at each of the three cell samples, plus six readiness flags.
Every `EvaluateCell` resets all flags before replacing coordinates. A normal is
computed only at its first original request; readiness becomes true only after
checked arithmetic completes. The first At/degeneracy pass and later short-circuit
checks remain in the original order. Intersections, all six VF feature tests and
regularity consume const references to those same exact normals. EE traversal,
common-translation evaluation, feature ordering, work/depth limits and publication
remain unchanged. No cache is shared across workers or cells, and no cached-input
flag or cache address is supplied by a caller.

The native forecast/allocation/retained-alias checks already use
`sizeof(ExactScratch)`, so all six normals and readiness flags are counted before
admission. Caps are unchanged. Integer values have fixed 16,384-bit inline
storage and allocation-free copies. On the qualified 64-bit-limb Boost build,
normal construction itself stays below the Karatsuba threshold and adds no
per-pair allocation. Higher-degree multiplications can allocate temporary Boost
workspace even though the integer backend's allocator parameter is `void`; see
the bounded-arithmetic audit below.

`NormalReuseQualification.h` compares fixed compile-time recomputing and memoized
executors on the same validated canonical pair. It returns the unchanged native
`StoreResult` representation and separate saturating diagnostic counts, never a
physical receipt or a runtime mode. `NormalReuseTest.cpp` compares all result
fields against this original uncached oracle, then compares public owner and batch
publication, one/four-worker scheduling, deep/degenerate/extreme-coordinate cases,
cell/pair/call reuse, alias/cap failure and retry. The degenerate-first-sample case
requires the original midpoint crossing and exactly three first-use normals;
this specifically rejects eager six-normal evaluation. Forecast tests cover the
exact enlarged scratch size and byte-cap-minus-one admission. Operation counts
prove removed repeated calculations; throughput gains require measurement.

## Exact integer range and allocation audit (2026-09-24)

Scope: validated finite binary64 endpoints, represented affine paths, native DFS
cell depth at most 52 and midpoint sample depth at most 53. This concerns the
exact helpers in `RepresentedIntervalCrossing.cpp`, not other contact arithmetic.
No backend, arithmetic, resource limit or error handling was changed by this audit.

Let `M=2^1024` and `E=-1074-53=-1127`. Each sampled coordinate has magnitude below
`M` and a stored dyadic exponent at least `E`. `At` uses nonnegative integer
weights summing to `2^depth`, so both its aligned weighted operands and their sum
fit the same 2,151-bit coefficient bound before the final exponent adjustment.
No coordinate is converted through floating-point interpolation.

| Quantity | Degree | Magnitude bound | Maximum magnitude bits at its lowest exponent |
| --- | --- | --- | --- |
| Sampled coordinate | 1 | `< M` | 2,151 |
| Coordinate difference / edge | 1 | `< 2M` | 2,152 |
| Normal / cross of two differences | 2 | `< 8M^2` per component | 4,305 |
| Coplanar SAT axis `normal x edge` | 3 | `< 32M^3` per component | 6,458 |
| Largest predicate comparison | 4 | `< 384M^4` | 8,613 |

The degree-four maximum covers point-in-triangle orientation, segment normal
squares/parameter comparisons and coplanar SAT projections. `RegularCell` only
forms `4*n(mid)-n(lower)-n(upper)`, bounded by `48M^2`. Coordinate comparisons
and common-translation checks are degree one. There is no exact division or
unbounded denominator refinement in these helpers.

`Add` aligns to the smaller existing exponent; at degree `k` it remains at least
`kE`, so the aligned operands as well as the result obey the bounds. Zero shortcuts
do not lower exponents. Negation preserves magnitude; multiplication adds degrees
and exponents; integer scaling is limited to interpolation weights or four.
Exponent values stay between -4,508 and 3,884, with alignment shifts at most 8,392;
these fit the observed 32-bit `int`. Time shifts are at most 53 bits in `uint64_t`.
The largest multiplication requests at most 136 64-bit result limbs (8,704 bits),
including whole-limb rounding, below the 256-limb / 16,384-bit checked capacity.
Thus admitted inputs cannot exhaust this integer range through these primitives.
This does not remove any capacity, work, depth, degeneracy or unsupported-motion
failure, nor establish that all caught `ExactArithmeticRange` errors are numeric.

Dependency evidence on this workstation: `/usr/include/boost/version.hpp` reports
Boost 1.74.0 (`BOOST_VERSION=107400`). The linked specialization uses 64-bit
`unsigned long long` limbs; `cpp_int/cpp_int_config.hpp:61-65` selects them with
128-bit intermediate support. `cpp_int/multiply.hpp:82-86` sets the default
Karatsuba cutoff to 40 limbs, with no override in the qualified CMake flags.
Degree-one operands of `Normal` have at most 34 limbs, so its products use the
nonallocating schoolbook path. Fixed-backend copy constructors/assignment are
`noexcept` and copy only inline storage and metadata; reference-parameter cleanup
does not remove an arithmetic or allocator failure point.

The broader no-heap claim is false for this Boost version. Its fixed-precision
`setup_karatsuba` in `cpp_int/multiply.hpp:265-309` aliases values through an
allocator-backed variable-precision type and obtains temporary workspace.
`cpp_int.hpp:290-293` calls `allocator().allocate(len)`. The qualified binary also
contains this specialization and calls to `operator new`. Valid degree-two
operands can exceed 40 limbs, so later degree-four predicates can reach this path.
A temporary allocation failure is caught by the existing `catch (...)` and
reported as `Unresolved/ExactArithmeticRange`; the numeric bound does not exclude
that failure. The normal cache preserves the remaining higher-degree evaluation
order and existing forecast/guard policy; this audit does not qualify a complete
library-temporary allocation budget.

Single-sample common-translation evaluation and skipping EE feature work after
all six VF tests found a canonical VF minimum remain deferred. Their geometric
results are invariant, and `VertexFace` sorts before `EdgeEdge`, but skipping
allocator-backed multiplications could remove a later resource failure. Absolute
resource-error equivalence is not established by the integer bound. Revisit the
allocation contract and qualify any such changes independently. Recheck this
audit after changes to Boost, limb width, cutoff, path depth or predicate degree.

## Relative coordinates and two fixed face axes

After the original sampled intersections, sampled degeneracy rejection, both
`RegularCell` proofs and world AABB test, production may certify another strict
whole-cell gap. It first subtracts the affine path of the lowest immutable vertex
key in the canonical first facet and tests coordinate endpoint hulls. If needed,
it projects those same relative endpoints on the first and second facets' exact
normals at the **lower sample**, in that order. The axes are fixed throughout the
cell. No edge-cross or coplanar in-plane axis search is added in this slice.

For a fixed axis n and common affine reference r(t), every vertex projection
n dot (x(t)-r(t)) is affine. A strict gap between endpoint hulls therefore separates
both triangles at every simultaneous time. Equality never certifies separation.
The original nondegeneracy requirement remains mandatory. Selecting the reference
by immutable source key makes it independent of winding; reversing a face normal
cannot change the symmetric strict-gap result. Common-translation handling
explicitly uses the legacy cell path because it already has a complete certificate.

A private immutable `ExactProjectionDomain` is derived from the actual pair's 36
endpoint components. It mirrors `Exact(double)` exponents, including zeros and
unnormalized mantissas. With D=max_depth+1, B=53+(emax-emin)+D bounds sampled
coordinate integers. Additional proofs run only when 2B+3 <= (cutoff-1)*limb_bits.
On Boost 1.74 / 64-bit limbs / cutoff 40 this means B<=1246, so degree-two operands
occupy at most 39 limbs. All added products stay below the allocator-backed
Karatsuba path, and the native 16,384-bit range proof still applies. The supported
library version is explicit; re-audit another version before enabling it. The
entire added bundle is disabled for an unsupported/wide domain. Such inputs retain
the original proof traversal and error policy. No caller supplies a valid flag.

This is an improvement in proof completeness and work, not a promise of identical
old unresolved statuses. Native `RelativeSeparationTest.cpp` covers bounded exact
gaps, touching and one-ULP offsets, true non-dyadic crossings, sampled/interior
degeneracy, an anchor-sensitive deformation/permutation case, domain boundaries,
wide fallback, scheduling, work limits and failed-publication retry. The private
`CompareRelativeSeparation` adapter runs one executor with two compile-time proof
choices; runtime production is fixed to the new choice. Diagnostic counters are
separate from physical proof work. A source-authenticated app fixture retains its
captured 4,095-visit exhaustion while comparing the legacy traversal with the new
one-visit face-normal certificate; accepted-owner/transition proofs remain separate.

Evidence motivating this slice is the guarded feasibility probe
`crash-work/reports/native-relative-probe-run-1.result.jsonl` in the workspace.
For source facets 2142381:1 / 2230072:1 it found a strict lower-face-axis gap and
passed both regularity/sample checks. The x+y-only separated control remains
outside this deliberately small axis set; genuine t=1/3 contact receives no gap.
No timestep, force law, thickness, source geometry, work cap or depth cap changes.

## Proposed native root interval filter (unqualified, 2026-09-24)

The isolated `native-interval-root-filter` branch adds a CPU-only conservative
binary64 proof before native exact interpolation and predicate evaluation. No
build, executable test, vehicle acceptance, or measured hit rate is implied by
this source change. It does not alter forces, thickness policy, the physical
clock, native pair ordering, capacities, or the exact fallback.

`RootIntervalFilter.cpp` reuses `Q4ContactBounds.h` and
`SurfaceMaterialBounds.h`; it introduces no new rounding implementation. The
native owner sorts each facet's three vertices by the existing complete source
key, preserving trajectories, and supplies a small copied endpoint value input.
This fixes arithmetic order under winding/permutation changes. Before this work,
an authenticated shared source vertex bypasses the filter: its identical
trajectory guarantees a shared point, so a strict gap is impossible. The bypass
only enters the original exact path, granting no contact or exclusion authority.
A contradictory shared trajectory still fails native identity validation before
any pair worker executes. A focused test checks full result equality, zero filter
attempts, and preservation of publication on conflicting identity. The helper exposes
only separated/inconclusive plus a represented axis for qualification, not a
physical receipt or a caller-selectable runtime policy.

For affine triangle edges e and f, the normal has quadratic Bernstein controls
`e0 x f0`, `(e0 x f1 + e1 x f0)/2`, and `e1 x f1`. Outward intervals enclosing a
strict common component sign at all three controls prove whole-interval
nondegeneracy of each triangle. Endpoint regularity alone never suffices. The
helper then encloses endpoint projections after subtracting one common affine
anchor path. Three coordinate directions and two represented lower-face
normal directions are tried in fixed order. A strict disjoint hull proves that
the zero-thickness triangles cannot meet anywhere in the interval. Face normals
are search directions only; their rounded construction carries no proof authority.
No finite-thickness owner, force, or exclusion is inferred from this certificate.

The existing path-derived `ExactProjectionDomain` admits the helper only where
skipping the native exact predicates cannot skip a Boost temporary-allocation or
checked-range failure. Wider inputs execute the original native path. Failed
interval operations, underflow, overflow, touching bounds, or ambiguous normals
return inconclusive; the original exact evaluation then starts unchanged.
Success consumes one native proof-work visit and produces the existing separated
record. Existing exact common-translation contact tags and witnesses are retained
on fallback. No extra retained scratch or device allocation is introduced.

Because the exact implementation does not require a particular floating-point
environment, the added filter checks its own environment. It currently enables
only on x86-64/SSE2 with `FLT_EVAL_METHOD == 0`, no fast-math, round-to-nearest,
and MXCSR rounding/FTZ/DAZ disabled. Other modes/platforms fall back. An admitted
proof temporarily establishes standard non-stop arithmetic with `feholdexcept`
and restores the complete thread-local environment and errno with `fesetenv`.
No callback or other solver code runs in that scope. A failed environment operation
cannot publish a filter certificate. The target retains no-FMA/reassociation flags and adds `-frounding-math` so
the compiler respects the dynamic floating environment around the proof.
CUDA portability of the reused arithmetic is not a qualified GPU filter claim.

`RootIntervalTest.cpp` compares complete native fallback/publication records and
checks successful certificates with the existing unlimited rational oracle,
extended to reconstruct midpoint normals and whole-interval projection hulls
independently of production interval formulas. It covers strict ULP gaps,
contact at non-dyadic time, sampled/interior degeneracy, permutation, extreme
arithmetic, non-RN and FTZ/DAZ fallback, flag/errno preservation, worker counts,
failed total-work admission, and retry. The old normal-reuse and relative-axis
qualification adapters explicitly disable this new filter so their original
operation-count and legacy-order oracles remain independent.

Promotion requires the owning native tests/source proof, frozen real-Yaris
replays, affected transaction/physical publication gates, and a bounded real-batch
measurement of filter successes, exact fallbacks, and wall time. A replicated
microbenchmark alone cannot establish vehicle throughput, and touching local
features will still need the exact path. No speedup is currently claimed.
