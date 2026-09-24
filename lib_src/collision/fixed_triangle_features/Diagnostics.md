# Discovery stage observations

`FixedTriangleFeatureLimits::enable_diagnostics` enables one value-only snapshot
per admitted call. The default is off. `diagnostics()` returns no observation
before initialization or while the owner is busy; a rejected concurrent entry
does not reset another call's snapshot. Geometry inputs retain their existing
immutable-borrow contract. No timing record authenticates them.

The coordinator measures seven disjoint scopes. InputLedger covers admission,
ledger construction, validation and compaction; InputSort surrounds the three
existing ledger sorts. TaskPreparation covers masks/counts/offsets. Geometry
includes the unchanged worker dispatch, numerical work and waits. ResultFold
covers the ordered error/result fold, duplicate checks and capacity admission.
OutputSort surrounds the two existing result sorts. Publication covers final
copies. These are host elapsed samples; Geometry is not aggregate CPU time.

There are only a constant number of clock reads per cohort and none on workers
or individual pairs. Discovery and the enclosing transaction reuse the same
monotonic clock primitive. The reader preserves errno. Failed or backward
samples increment explicit counters; invalid samples are never zero-duration
measurements. Accumulators saturate instead of wrapping. Clock behavior cannot
change geometry, capacity decisions, reports, work charges or publication.

A failed call replaces the diagnostic snapshot with its measured prefix while
preserving the independent previous complete geometric publication. A later
retry resets the timing record. Empty successful queries record admission and
empty publication. Enabled and disabled owners have identical retained storage;
the fixed snapshot is included by the existing `sizeof(Impl)` forecast.

The transaction enables child measurements with its existing diagnostic option,
then accumulates each returned last-call snapshot into the existing accepted or
candidate attempt. Child timing counters remain distinct from physical discovery
counts and outer inclusive Discovery time. The app observer can therefore show
both the inclusive stage and its internal breakdown without double-counting them
as separate total costs. This adds no new physical or scheduling authority.
