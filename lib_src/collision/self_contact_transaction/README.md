# Self-contact architecture and reading map

`SelfContactTransaction` composes fixed-facet self-contact into TL-FEA's existing
physical transaction. It owns bounded attempt scratch and contact participants;
it does not own a second nodal state, solver clock, or commit operation.
This page describes code organization. Run acceptance and resource allowances
belong in the workspace handover, not in this module contract.

## Start here

Read these interfaces in order before changing implementation:

1. [FENodalState](../../solvers/FENodalState.h): accepted/trial state, owner
   stream, view lifetime, commit, discard, and poisoned CUDA state.
2. [ShellBatchPublication](../../elements/ShellBatchPublication.h): coordinated
   structural-history publication and participant authority.
3. [SelfContactTransaction](../SelfContactTransaction.h) and
   [public types](../SelfContactTransactionTypes.h): startup forecast, accepted
   assembly, candidate sealing, receipts, and typed failures.
4. [Transaction.cpp](Transaction.cpp): accepted-state force/event assembly.
5. [Candidate.cpp](Candidate.cpp): prepared-state validation and interval
   policy. Its qualification census is a diagnostic path, not an alternative
   acceptance operation.

## Ownership and dependency direction

| Layer | Responsibility | Entry points |
| --- | --- | --- |
| Physical owner | Node state, rigid/CIN state, one time/epoch, additive assembly, final commit/discard | `FENodalState`, `ShellBatchPublication` |
| Source binding | Selected surface, fixed Q4/T3 facets, canonical feature identities and physical ownership | [SelfContactSurfaceBinding](../SelfContactSurfaceBinding.h), [FixedContactFacetBinding](../FixedContactFacetBinding.h), [SelfContactActiveUseBinding](../SelfContactActiveUseBinding.h) |
| Current source state | Actual activity/removal and current parent regularity | [SelfContactPhysicalActivity](../SelfContactPhysicalActivity.h), [SelfContactCurrentRegularity](../SelfContactCurrentRegularity.h) |
| Geometry primitives | Complete pair discovery, exact feature predicates, represented interval certificates | [SelfContactBroadphase](../SelfContactBroadphase.h), [FixedTriangleFeatureDiscovery](../FixedTriangleFeatureDiscovery.h), [RepresentedIntervalCrossing](../RepresentedIntervalCrossing.h) |
| Physical response | Accepted force events, equal/opposite response, stiffness/time-step bounds, rigid/CIN transfer | [SelfContactForceAssembly](../SelfContactForceAssembly.h) |
| Contact composition | Joins source, geometry, response, and physical scratch participation | `SelfContactTransaction` |
| Application composition | Selects vehicle/profile, orders wall and self-contact alongside structural participants, writes accepted output | robo-dyna `case/vehicle_self_contact/` and `case/vehicle_dynamics/` |

Chrono supplies application infrastructure and visualization outside this
transaction. Legacy DEME/hydroelastic collision classes coexist in `collision/`
but are not dependencies of this self-contact transaction.

The native affine interval certificate is implemented in
[RepresentedIntervalCrossing.cpp](../RepresentedIntervalCrossing.cpp).
Its private [batch operation](../represented_interval_crossing/Batch.h) owns
path-roster authentication and its synchronous worker lifetime.
[CrossingBatch.h](CrossingBatch.h) is the transaction adapter: it bounds native
slices while preserving the complete feature-discovery cohort and canonical
result order. The native implementation is in `lib_src/collision/`, not
`lib_utils/`. Shared storage/identity utilities such as
[BoundedArena](../../../lib_utils/BoundedArena.h),
[BoundedStartupArray](../../../lib_utils/BoundedStartupArray.h), and
[SourceIdentityIndex](../../../lib_utils/SourceIdentityIndex.h) remain separate
from contact physics.

## One physical attempt

1. The common owner begins a trial. Structural contributors and configured
   contact contributors assemble accepted-state force and stability data.
2. `AssembleAccepted` captures authenticated activity and coordinates, streams
   complete canonical pairs, builds the accepted event ledger, and stages
   force/STI through the existing owner stream.
3. The common dynamics coordinator seals assembly, executes its structural/CIN
   step screen, advances the owner, and prepares structural candidate history.
4. `SealCandidate` authenticates those prepared fields, builds motion bounds,
   and validates complete continuous geometry and physical feature ownership.
   Success produces the self-contact receipt required by common publication.
5. Common publication commits all participants or discards the complete
   attempt. A contact-local `DiscardTrial` alone does not discard the owner or
   structural history.

A native crossing result can establish an intersection at one time. It does
not by itself establish admissible contact over the whole interval. Local
topology, persistent contact ownership, and thickness obligations remain the
transaction's responsibility. This distinction is essential when modifying
candidate diagnostics or reusing a geometry result.

## Internal navigation

| Files | What to look for |
| --- | --- |
| `Arena.cpp`, `Layout.cpp`, `Limits.cpp`, `Storage.h` | Retained storage, forecasts, phase/state layout, private qualification access |
| `Streaming.cpp`, `TaskMask.cpp` | Complete canonical facet-pair streaming and source-incidence task masks |
| `Source.cpp`, `Values.cpp` | Accepted feature admission, deterministic keys, identity and candidate-policy values |
| `Transaction.cpp` | Accepted event census and staged force publication |
| `Candidate.cpp` | Owner authentication, swept candidates, continuous policy, sealing, diagnostic census |
| `RigidSweep.cpp` | Exact rigid/affine path coefficients, directed Bernstein bounds, interval topology and accepted-owner coverage; also used for affine nodal cases |
| `FinalizedCoverageLedger.h` | Private lexical lookup ranges over the already finalized feature-first accepted ledger; raw geometry calls retain the full-scan oracle |
| `LocalContact.*`, `TranslatedLocal.*`, `CandidateExclusions.*` | Local topology, exact translation normalization, authenticated support exclusions |
| `CrossingBatch.*` | Native crossing slice admission and diagnostic adaptation |
| `Qualification.cpp`, `CandidateFailureCapture.*`, `QualificationReceipt.h` | Authenticated diagnostic access and frozen failure evidence; no independent commit authority |

Broadphase and force/STI assembly use CUDA. Feature discovery, exact interval
crossing, and Bernstein policy certification currently run on the host.
Canonical ordering and fixed-order force gathering provide determinism;
thread scheduling and elapsed time are not acceptance criteria.

## Tests and changes

The detailed [transaction qualification guide](../../../lib_utest/qualification/self_contact_transaction/README.md)
records numerical contracts, test scope, and focused harness instructions.
Start with value tests, medium coupons, and source/syntax checks. Geometry
changes also require the [native interval tests](../../../lib_utest/qualification/represented_interval_crossing/README.md)
and relevant frozen source cases. GPU transaction/determinism and
[physical publication tests](../../../lib_utest/qualification/physical_publication/README.md)
cover device effects, discard/retry, and common-owner integration. Full vehicle
gates belong in robo-dyna after focused qualification; they are not a routine
documentation check.

## Deferred structural cleanup

The public components already separate source, geometry, response, and
publication. The main readability debt is inside three large implementation
files, rather than a need for another solver hierarchy:

- Split `RigidSweep.cpp` by exact coefficient arithmetic, Bernstein geometry,
  topology, and accepted-owner coverage after preserving focused proof tests.
- Split `Candidate.cpp` by motion preparation, per-pair policy, and census.
  Share the production per-pair policy with future diagnostic census work:
  the current census counts raw crossings before the final policy used by
  production and is not a complete production-policy oracle.
- Separate qualification declarations from `Storage.h` and categorize the
  broad `Values.cpp` helpers without expanding public authority.

These are future refactors, not changes made by this documentation cleanup.
Preserve arithmetic order, first-error priority, cohort/seam ownership,
work accounting, view lifetimes, and atomic publication during extraction.
Do not combine a numerical change with a file move, or infer acceptance from
a source-text check alone.

## Accepted owner range lookup

Candidate sealing borrows the transaction's finalized accepted ledger only after
same-assembly, owner, epoch and attempt authentication. The lexical borrow is
noncopyable and cannot be constructed by callers; it is never read after a failed
seal discards scratch and never retained for another attempt. Finalization already
sorts certificates by complete geometric feature before physical parent ownership.
No new retained index or device buffer is allocated.

A VF coverage owner must reference one of the candidate pair's six vertices; an EE
owner's first canonical edge must be one of its six edges. At most twelve prefix
searches therefore find a complete superset of possible owners. Prefix keys use
all native source identity fields. The fixed stack ranges are unioned in original
certificate ordinal order, and the unchanged exact owner builder applies its full
feature/active-owner/seam tests. Distinct physical owners are never deduplicated.
The original 64-owner bound, source-order sort, ambiguity checks, certificates,
Bernstein geometry, subdivision/work accounting and failure policy are unchanged.
Raw pointer/count coverage calls keep the original full scan, including unsorted
qualification arrays and small synthetic exclusion arrays.

`OwnerRangeCases.h` compares every result field against that raw full scan and
checks seam/owner provenance, native VF strata and both orientations, EE identity,
malformed owners, duplicate source order, 0/64/65 owners, work/depth/geometry errors,
deferred exclusion behavior and ordinal preservation. Deterministic lookup/visit
counts show reduced search work on a sparse ledger; no wall-time speedup is claimed.

## Shared-vertex proof ordering

After the existing endpoint classification, source-key, shared-coordinate-path
and whole-cell nondegeneracy checks, the shared-vertex geometry result is the
logical OR of two existing proofs: no nonlocal polynomial transition roots, or
strict opposite cone signs. Production evaluates the same bounded 20-direction
cone first. A successful cone skips four VF, five nonincident EE and four incident
EE polynomial tests; a failed cone retains the unchanged polynomial proof. No
proof work/depth, axis order, geometry, thickness, tolerance or resource cap changes.
Cone failure with polynomial success performs extra cone work, so this reordering
requires measurement on representative families before any throughput claim.

The private `SharedVertexProofQualification.h` adapter instantiates both orders at
compile time and returns complete geometry results plus independent saturating
operation counts. It cannot select a production order or mint a physical receipt.
`ProofOrderCases.h` and `ProofOrderCoverageCases.h` compare every result field,
including witnesses, source/certificate order, unresolved cells, work/depth flags
and digests, and check that cone success actually eliminates the polynomial tasks.
The common field-wise test utility is `NonlinearResultAssertions.h`.

This equivalence concerns geometry outputs and solver state under the existing
floating-point execution contract. Arithmetic and `nextafter` can change errno or
sticky floating-point flags, and skipped operations need not set the same flags.
The solver does not consume those flags as geometry authority; trapping-mode or
errno equivalence is not asserted. Endpoint classification remains in its original
position in this first optimization.

## Optional host substage observations

`SelfContactTransactionConfig::enable_diagnostics` defaults to false.
`SelfContactTransaction::diagnostics()` copies bounded, nonphysical observations
of the last accepted assembly and candidate sealing calls. The public value
header has no CUDA dependency. No receipt, configuration identity, policy digest,
force, proof work, limit, acceptance or publication decision reads these values.
The fixed storage is included by the existing `sizeof(Impl)` host forecast.

Private `DiagnosticAttempt` reuses the monotonic-clock, errno-preservation,
valid-sample and saturating-counter pattern from robo-dyna's
`benchmarks/stage_timing/StageTimer`. It deliberately does not add a TL-to-app
link or make standalone app host tools depend on TL. Only private qualification
can replace the per-instance clock; production always uses `CLOCK_MONOTONIC`.
Disabled observation reads no clock and performs no aggregate counting.

The single coordinator switches scopes at cohort boundaries, never inside a
pair/feature arithmetic loop. Stages are disjoint host elapsed time, including
existing worker/device waits; there are no new CUDA events or synchronizations.
Setup includes authentication, source snapshots, regularity and broadphase.
Accepted filtering includes streaming, pair rejection and task masks; discovery
is the existing worker/reduction call; event assembly includes feature policy,
identity census, ledger merging and sorting; force assembly and final participation
are separate. Candidate filtering includes streaming, prism and nonlinear root
proofs; discovery is separate; residual includes persistent/translated proofs,
nonlinear owner coverage and path packing; native crossing includes the complete
bounded adapter; policy includes intersection/edge checks, linear owner coverage,
outcome validation and folding. Finalization covers complete census checks and
participation sealing. Empty terminal stream calls still form filtering samples.
Candidate task-mask construction is included with the adjacent policy bookkeeping.

Discovery counters sum fields already returned by every executed call, including
an optional second accepted census verification pass. They distinguish raw versus
deduplicated records. Native counts derive from the existing compound batch report:
completed slices plus a last invoked failing slice; an empty native slice counts
as one batch and zero pairs. `native_work` sums admitted native work, excluding
linear policy coverage. These counts are distinct from the older
`exact_crossing_pairs` policy cohort and mixed `exact_crossing_work` summary.

Every attempt resets its own record; a new accepted assembly also clears the old
candidate record. Requested owner/base-epoch/attempt values are explicitly marked
unauthenticated until ordinary owner checks succeed. Rollback/discard retain the
record for inspection. Normal failure or exception closes the active scope,
sets `finished`, leaves `succeeded` false and marks `counts_complete` false: a
traversed prefix never claims the complete model census. A copied observation
never becomes publication authority, including after a later commit or discard.

Clock read failures/backward samples omit elapsed time and increment explicit
fault counters. Consumers must use `calls`/`valid_samples`; unavailable time is
not a zero-duration observation. Counter overflow saturates and marks the record
incomplete. Timing and count faults never change a physical result or first error.
No per-pair logs, dynamic storage, worker instrumentation or second startup pass
are introduced. Host diagnostics tests exercise clock faults, unwind, saturation,
actual native split/empty calls and failure counting. CUDA coupons compare force
bits, accepted/candidate receipts, policy outcomes and committed owner state with
observation disabled, enabled and an always-failing clock, and verify failed
candidate rollback plus retained/reset observations across retries.

## Bounded affine shared-vertex cone directions

The dedicated local-topology phase may now search a complete finite family of
constant separating directions after both existing alternatives fail. The original
20 directions, their order, and the polynomial proof keep every existing successful
short-circuit. Source-key equality, identical shared-coordinate paths, endpoint-local
classification and whole-cell nondegeneracy remain prerequisites. Unmasked positive
thickness separation is still checked by `CertifyQuadraticLocalContact` before this
phase; the new helper supplies no force, friction, ownership or exclusion shortcut.

For an affine pair, form eight signed endpoint arms: A's two nonshared arms at each
endpoint and the negated arms of B. A strict constant separator exists exactly when
the origin is outside their convex hull. The closest point p then satisfies
p dot r >= p dot p > 0 for every ray. Its supporting face has dimension at most two,
so a vertex, edge or triangle subset recovers a suitable direction. `ConeDirections`
streams those candidates: a, e cross (a cross e) for e=b-a, and (b-a) cross (c-a).
This is at most 8+28+56=92 additional directions, with only eight rays and a cursor
retained. The original 32-slot candidate array is unchanged.

Candidate construction uses represented binary64 arithmetic and grants no authority.
The original `DotPolynomialAxis` and strict Bernstein signs verify each proposed
axis against all four arm trajectories. Zero, nonfinite or uncertain directions
cannot certify anything; there is no epsilon, geometry perturbation, or cap increase.
The finite-family completeness statement is exact-arithmetic mathematics, not a
promise that rounded candidates always find an axis. Extremely scaled but feasible
sets can remain inconclusive and follow the original failure path.

The new search is enabled only at depth zero/path zero of an authenticated affine
`local_topology_only` traversal. `CertifyQuadraticFacetPolicyCoverageImpl` invokes
`CertifyQuadraticLocalContact` once; that method invokes local topology at most once.
Recursive children have positive depth. Ordinary accepted-ledger and synthetic
exclusion continuations use `local_topology_only=false`. Thus one complete policy
attempt generates at most 92 additional directions, including its fallbacks.
No external budget or caller-supplied valid-geometry flag is introduced. Subdivision
work/depth accounting and failure priorities remain unchanged apart from newly
proved local geometry.

Qualification adds independent unbounded rational feasibility and exact arm-dot
helpers in `ConeDirectionOracle.h`, iterator ordering/rank/touch/overlap tests,
rotated/scaled families, bounded inconclusive extremes, a generic coplanar geometry
missed by both old proofs, unchanged old successes, real curved-crossing negatives,
and source/path rejection. A common invertible affine rotation has no constant root
separator and checks that the exhausted 92-direction search runs once even when
subdivision follows. The app owns the separately pinned gate12 fixture/full-policy
regression; no original vehicle IDs or fixture axis appear in production.

This source slice is unbuilt and unexecuted at authoring. Owning host/source/CUDA
gates, frozen replay, and real vehicle acceptance remain required before promotion.

## Bounded curved shared-vertex cone directions (unqualified source)

Curved paths may use the same final strict cone certificate with a larger, still
fixed candidate family. The existing affine helper and its eight-ray ordering are
unchanged. A curved root instead forms twelve signed arm controls: the three
quadratic Bernstein controls for each of A's two arms, and the negated controls
for B's two arms. Interval-control midpoints select directions only. No midpoint,
endpoint, iterator, or feasibility result grants physical or geometry authority.
Each proposed direction is checked by the same `SharedVertexAxisSeparated`,
`DotPolynomialAxis` and strict whole-interval directed Bernstein sign tests.

`ConeDirectionStream<12>` retains twelve rays and a cursor and streams at most
12 singleton + 66 edge + 220 triangle directions, or 298 total. For exact control
vectors, the same closest-convex-hull-face argument gives a complete finite family
for strict constant separation of those vectors. Interval midpoint selection and
rounded candidate arithmetic are heuristic. Even feasible exact curves may have
no strict control-hull separator, or the interval enclosures/rounded directions
may fail to find one. In every such case, the original subdivision/failure path
remains available. There is no completeness claim for all curved local geometry.

Only the dedicated local-topology root may attempt this extension, after existing
coordinate, source/shared-path, whole-cell regularity, old cone and polynomial
proofs. Endpoint-local premises are retained. Exactly affine input takes the old
92-direction branch; nonaffine input takes the new 298-direction branch. Children
and ordinary accepted-ledger/synthetic-exclusion fallbacks cannot rerun either
search. Thus the additional family is at most 298 per complete policy attempt,
not 298 per cell. Positive-thickness/residual obligations remain separate and
unchanged, as do proof work/depth limits and every previous successful winner.

Private `CompareCurvedConeSearch` compares the accepted affine-only executor with
the full executor; there is no public runtime mode. Tests include an exact narrow
curved control family for which all endpoint-generated directions fail, genuine
interior nonlocal contact with identical local endpoints, uncertain coefficients,
rotations/extreme scales, affine/curved old winners, source/budget rejection, and
an exhausted root search followed by unchanged recursion. The unlimited rational
oracle is independent of directed arithmetic and candidate construction.

This slice is source-only, unbuilt and unexecuted. It was motivated by a new curved
pair reported in gate13, but that failed pair has not yet been captured by the
native failure observer and is not represented by these generic tests. No claim
that gate13 is resolved is permitted without its authenticated fixture and full
policy replay. Capture repair, owning qualification and final vehicle acceptance
remain mandatory. No vehicle IDs or fixture-specific directions enter production.
