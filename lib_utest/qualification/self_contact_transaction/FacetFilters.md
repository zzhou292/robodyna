# Optional CUDA facet filters and compact candidate queries

The optional backend defaults off. The selected numerical Batch, kernel and
arithmetic are unchanged; the compact chunk work starts from qualified TL
3c0ee611. Its older span adapter passed full correctness and exact vehicle replay,
but the real first interval was 46.21% slower than the selected CPU contact path.
This compact revision is unqualified until its owning tests and transfer-inclusive
adapter benchmark pass. It does not integrate the separate GPU-native executor.

## Ownership and storage

One private FacetFilters owner composes the numerical Batch. The copied scene is
160F bytes on the selected ABI; existing device pairs/results add 12K device
bytes and host readback adds 4K. Compact chunk staging adds K copied pair records
and K uint32 original-to-packed slots: 12K host bytes, 48 KiB at K=4096. Existing
limits require K<=UINT32_MAX, so UINT32_MAX is the unused-slot sentinel. Checked
arena alignment and sizeof drive the full owned/startup forecasts. There are no
new device buffers, workers or allocation/query-time heaps.

Disabled execution retains the scalar path without optional O(F+K) storage or
device allocation. Fixed control/report bytes remain honestly charged by current
preflight. Unsupported startup arithmetic chooses lifetime CPU fallback before
filter allocation/commands; exact initialization mode and allocation telemetry
distinguish that from the requested conservative reservation.

Accepted scene upload and both census passes are unchanged. Scalar and CUDA
accepted classifiers still feed the same original validation/compaction fold.
Earlier numeric-invalid rows retain priority over later malformed metadata.

## Candidate chunk contract

CandidateScene copies/uploads authenticated scene data once. BeginCandidateChunk
now returns a checked Report and prepares one compact numerical query. It scans
only this materialized chunk, preserves original order, and packs precisely the
metadata-valid rows whose existing classifier returns LinearNodalV1. Invalid
future ordinals/parents get no slot and are not reported eagerly. Nonlinear work,
source validation, canonical ordering and policy remain in the original serial
Candidate.cpp fold. Numerical-invalid rows remain cached PairResults until that
fold requests the original ordinal.

PrismAt performs no CUDA query. It checks lifetime, original ordinal, mapped slot,
exact copied pair identity and scene generation before consuming a row. Empty and
no-linear chunks launch no query. A rejected chunk replacement revokes the previous
borrow after range/alias inspection and before any possible later consumption.
No rejected input is dereferenced or packed. Discard revokes scene/chunk aliases;
all owned ranges participate in OutputDisjoint.

An unsupported environment at preparation uses CPU for that chunk. An environment
change during consumption revokes the mapped cache and selects CPU for the rest
of the chunk, even after RN is restored. A following chunk may use CUDA. This
explicit lifetime rule permits at most one query per chunk; restoring the
environment never causes a second submission for that chunk.

A real CUDA/transfer failure permanently poisons the adapter and cannot become
CPU fallback. Candidate preparation failure is recorded as CandidateChunkBeforeFold,
with the original chunk offset/count and no invented offending pair. Its original
backend status/message are retained. Device failure may precede that chunk's
ordinary serial errors; ordinary source/geometry/budget failures preserve their
old positions and exact reports. This nonphysical error context is separate from
receipt/publication authority.

## Qualification

Existing actual-owner tests remain the force/STI/receipt/policy/state oracle for
scalar versus optional execution, discard/retry, exact capacity and nonlinear
work exhaustion. New value/adapter cases cover copied ordinal maps, duplicate and
reversed pairs, mixed nonlinear/excluded gaps, malformed future metadata,
per-row invalid geometry, empty/no-linear batches, stale identity, rejected/aliased
replacement, sticky environment fallback, and poison before chunk consumption.

The query-only qualification counter observes actual H2D pair transfers/cardinalities
after scene setup. Existing transaction stage counts are not GPU launch counts.
The maintained benchmark builds the same driver against the old span and compact
source, uses actual adapters and the same retained source fixture, verifies every
row outside timing, and labels synthetic routing patterns. It includes packing,
admission, transfers, kernel, readback and mapping costs. No next vehicle probe is
admitted until this evidence shows a clear benefit without changing physics,
candidate membership, limits or the original serial nonlinear/native work fold.
