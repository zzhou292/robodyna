# Fresh compact T3 activity validation

This source is prepared for owning qualification; numerical/CUDA tests have not
run yet. Base: TL 95664290 with shared ActivityLayout extraction f0fe2c86.

Mapped activity now validates all resident records on the owner's stream, then
copies bounded role/active/error packets. It reuses the complete existing force,
one-point, mixed-section and failure predicates. The five global GPU phases are
one-point, mixed, failure, force and point/force identity. Host role agreement
remains between the last two. Source-parent error priority and the original
force-derived versus explicitly supplied point time/epoch remain distinct.

Every query is fresh. There is no cached verdict or all-active assumption.
Public capacity, owner/token, receipt, source and output-range preflight remain.
Static shape/source eligibility only selects the new path; failure falls back
to the unchanged complete reader and does not publish a reordered error.
Nonphysical and complete history/section readbacks retain their existing path.
A call-local pointer identifies the final validated flag packet; it never escapes
the public query. Point/force identity can reuse the force scratch packet because
final caller flags come from the separately retained failure/point activity packet.

Storage adds one shared ActivityMemory packet to the mapped arena and three
bounded host packets, including their actual control-allocation allowances.
No per-query allocation or second owner/clock is introduced. The selected V5
T3 query formerly copied 38,000,984 bytes (21,301 * 1,784); with its genuine
one-point sidecar the new path copies five * 21,305 bytes. These are byte counts,
not an execution or speed claim. Real V5 qualification must prove compact path
admission using existing API/kernel traces after exact accepted-state comparison.

## Qualification inventory

- Seven host tests: complete frozen-loop comparison for all five section roles,
  initial/later states, 24 record/encoding/identity faults, cross-phase priority,
  reversed parent order, packet/cap boundaries, valid inactive constant/TAB1,
  no-point storage, 30 reused failure cases, curves/continuation/rate policies.
- Six CUDA predicate tests: the same complete frozen oracle, counts across
  127/128/129 boundaries, both selectors, repeated-query mutation/freshness,
  inactive/TAB1/no-point and relocated tabulated curves.
- Eight actual-owner tests: six accepted/prepared intervals and discard/retry,
  every remaining copy/drain and pending/launch error, malformed packets,
  private output aliases and foreign/stale/cap preflight, prepared failure,
  unchanged full-history APIs, exact/one-short forecasts, actual device-history
  corruption and wrong history identity against frozen predicates.
- Source proof authenticates ten complete old files, generated original loops,
  unchanged full-history/preflight bodies and the explicit phase schedule.

The numerical fixture uses prescribed retained values to test validation, not
another force solver. Actual-owner tests use the existing physical publication
fixture and genuine public typed histories. Frozen loops replace only transport
and immutable source lookup seams; shared original numerical leaves remain.

After focused tests, required affected gates are the existing QEPH mapped
activity tests (shared shape helper regression), T3 one-point resident/native,
shell parent activity and Q/T mapped tests. Existing old transfer-count tests
name the superseded full-copy behavior and must not be represented as current
compact traffic. Full-history and nonphysical legacy behavior still need checks.
Finish with the real two-interval owner and exact 101-step archive/physics gate,
then fresh API/kernel and stage timing. No actual vehicle performance claim
follows solely from the small predicate coupons.
