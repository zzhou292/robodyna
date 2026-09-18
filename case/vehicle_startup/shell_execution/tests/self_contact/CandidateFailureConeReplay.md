# Pinned shared-vertex cone regression

This host regression preserves the first rejected pair from observed vehicle gate8. It is authored but has not yet been built or run. It is diagnostic geometry/publication qualification, not a physical commit, restart or complete vehicle acceptance.

The fixture is external and immutable:

- Directory: `crash-work/fixtures/wall-self-observed-failure-8`
- Manifest SHA-256: `3b2ab9990ad1dbf5c98a27c28420d2fb9e0847b2226337eb519a69e8d3559da9`
- Pair SHA-256: `0bf7a5002eb07d8c9367de2d8174410a52566769d1eed5eabf74f00277cabed7`
- Native pair: `2344779:0 / 2344792:0`; shared source vertex `2333970`.
- Captured state: accepted epoch0, 200ns candidate, zero quadratic coefficients, unchanged activity, no relevant accepted owner in the equivalent compact ledger.
- Original rejection: raw affine work exhaustion; standalone local/policy geometry did not certify. This is not the tiny-edge representation failure or exact common translation.

`CandidateFailureConeTest.cpp` reuses `ReplayCandidateFailure` for bounded, caller-independent SHA pinning and codec/source/phase validation. It checks the native endpoint intersections and residual thickness certificate, reconstructs the exact affine native query only after authenticating zero curvature, and retains its raw unresolved result. The corrected `LocalContact` and `PolicyCoverage` must each certify complete local topology in one root cell. The existing final publication validator must then classify the pair as a local intersection with no physical owner ordinal. Source and roster hashes must remain unchanged.

The numerical correction is owned by TL-FEA's isolated `work/self-contact-cone-diagonals` branch: six generic coordinate-plane diagonal axes appended to the existing bounded shared-vertex cone search, followed by the unchanged strict Bernstein verification. This app test introduces no geometric proof or physical exception. Its projection to the final validator mirrors the existing candidate local-proof branch.

Build target: `robo_dyna_candidate_failure_cone_check`.
CTest: `vehicle_candidate_failure_cone_replay`.
GoogleTest: `CandidateFailureConeReplay.PinnedAffineSharedVertexPairRetainsCompleteLocalPolicy`.

The test requires `ROBO_SELF_CONTACT_CONE_FAILURE_MANIFEST` to name the above `failure.json`; hashes are fixed in test source. Run it only through the normal workstation guard after a coherent app/TL build. The separate `vehicle_candidate_failure_native_replay` test still uses its earlier exact-translation artifact and existing environment variables; do not substitute this fixture into that test. No frozen artifact is rewritten by either test.
