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
