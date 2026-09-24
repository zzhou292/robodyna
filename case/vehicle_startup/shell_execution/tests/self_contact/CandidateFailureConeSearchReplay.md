# Epoch-one affine cone-search regression

Source checkpoint, 2026-09-24: authored but unbuilt and unexecuted. This host
qualification is a frozen geometry plus complete policy/publication replay. It
does not commit a physical interval or establish a complete vehicle acceptance.

The test uses the immutable failure from vehicle gate 12:

- Caller-selected manifest: `crash-work/fixtures/wall-self-observed-failure-12/failure.json`
- Manifest SHA-256: `f9f22f09f005775a2d23a8289eeb314a2043546fb4ca65a1eba81b8a695b6053`
- `pair.bin`: 12,352 bytes, SHA-256 `ee5aa0f90a24b74bf733c70863403f164e85aa26222179972f84640b259bb09d`
- Facets `2352845:1 / 2352854:1`, shared source vertex `2362259`.
- Actual accepted epoch 1 at 200 ns, prepared epoch 2 at 400 ns, attempt 3. The first
  physical interval committed; the second was discarded after this failure.
- Both physical half-thicknesses retain binary64 bits 4563130729260870543
  (approximately 0.00119 m). All affine curvature coefficients are exactly zero.
- Original candidate rejection: unresolved/work-exhausted. The raw affine engine
  has the ordinary time-zero shared-vertex witness; the unresolved reason is
  produced by later whole-interval policy, not by a missing raw first witness.
- Captured standalone compact and full-ledger policy replay: missing accepted
  owner, work 51, deepest 20, depth exhausted. Full ledger 32,488 owners, equivalent
  compact ledger 0 owners. The blob baseline nonlinear status/work were unreported;
  the test preserves that distinction instead of relabeling zero as measured work.

`CandidateFailureConeAssertions.h` contains the existing gate 8 replay checks,
extracted without changing them. Both gates reuse the bounded `ReplayCandidateFailure`
reader, frozen endpoint intersections, masked residual thickness certificate,
raw native witness, complete `CertifyQuadraticLocalContact`, complete
`CertifyQuadraticFacetPolicyCoverage`, and final `ValidateCandidatePublications`.
The corrected source must certify local topology in one root cell and publish
`ExcludedLocalIntersection` without inventing an accepted force owner. Source and
roster hashes remain unchanged. The gate 8 CTest and environment variable stay
separate from this regression.

Gate12 additionally verifies the actual phase, physical thickness, captured
failure metadata and an independent exact rational separating-axis witness. It
reuses TL's qualification-only `ConeDirectionOracle.h::ArmDot` to subtract the
original coordinate bits exactly. The test-only rounded axis
`(-0x1.ab0adbb1622d2p-1, 0x1.033b5b69501b2p-1, -0x1p+0)` has strictly opposite
signed projections for all four nonshared arms at both endpoints. These are
projections, not Euclidean distances. The TL production search must derive a
geometry-dependent direction generically; neither these source IDs nor this axis
are permitted in production code.

A negative mutation moves one prepared B arm onto A's exact nonshared vertex
while retaining distinct source keys. Exact endpoint intersection classification
must then require nonlocal admission, and complete positive-thickness policy
must remain uncertified. The copied mutation never changes the frozen fixture.

Build target: `robo_dyna_candidate_failure_cone_search_check`.
CTest: `vehicle_candidate_failure_cone_search_replay`.
GoogleTest: `CandidateFailureConeSearchReplay.EpochOneAffinePairNeedsGenericSearchAndCompleteLocalPolicy`.
Environment: `ROBO_SELF_CONTACT_CONE_SEARCH_FAILURE_MANIFEST` must point to the
manifest above; the expected hashes are fixed in test source. Build only against
the coherent TL generic affine-cone extension and run through the workstation
guard. The existing gate 8 replay must also pass after the shared-assertion extraction.
