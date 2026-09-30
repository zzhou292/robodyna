# CW1 raw reader boundary

Source preparation only; five focused host functions are staged and unexecuted.
This reader adds no scientific decision, native interval call, mechanics owner
or source-trust claim. Existing raw writer, collector and numerical model files
remain unchanged.

`ReadRawJob(directory, binding, output, error)` requires one exact cells/boost
identity, an externally supplied final-index SHA256, a separately supplied exact
provenance SHA256 and the remaining shared byte allowance. The final index binds
the complete ordered inventory. Neither an index hash nor hashes listed in a
self-provided provenance document establish trust in source/build inputs; that
authentication belongs to the external launcher and retained source checkpoint.

The reader checks every artifact's actual regular-file kind, bytes and hash,
rejects symlinks and uninventoried directory entries, and validates every earlier
progress inventory against the exact prefix then present. File names, finite
caps, producer budget, model/provenance links and finest-native contact links
are checked before publishing one reconstructed `RawReadResult`. It holds one
job and a temporary payload document, never six jobs. Each file remains bounded
by 32 MiB; all files including the index must fit the explicit remaining portion
of the shared 96 MiB cap. The caller accounts other raw/derived jobs separately.

The owning builder reconstructs the fixed model. Its canonical serialized fields
must match retained geometry, reference frames, native mass/inertia partitions,
contact measures, penalty bits/chain, wall coverage and full dictionary exactly.
Payloads are parsed with full binary64 precision and bounded iterative JSON;
duplicate/unknown fields, changed tuple/grid/dimensions, invalid certificates,
bad row/parent associations and nonfinite completed operands are rejected.
The finite full contact operator is recomputed from the stored finest native
matrix and owning contact kick. Declared sign-cone directions and completed
point masks are checked. No expensive native map is rerun.

A final index can close an incomplete numerical collection. Such a record loads
for diagnosis with `job.collection_complete=false`; a directory without a final
index cannot load. Explicit nonfinite marker kinds/bits are preserved only in
failed/partial operands, and every completed native column and published physical
sample remains finite. A complete collection may still have failed derivative
checks. Saved pass bits, residuals and budgets are retained as measurements;
the reader does not turn them into admission. The subsequent analysis must
recompute quotient/decision evidence from raw baseline and physical samples,
check native baseline identities, then apply the frozen full-state policy.

All expected read/validation failures return false and preserve the complete
caller output; the diagnostic string reports the failure. Files are read under
the same externally serialized assumption as ArtifactIO, without an atomic
filesystem snapshot or automatic resume claim.

Root wiring proposal: `qeph_wall_raw_read` comprises `WallRawRead.cpp`,
`WallRawReadFields.cpp`, `WallRawReadHeader.cpp`, `WallRawReadContact.cpp` and
`WallRawReadProbes.cpp`, linked to existing `qeph_wall_raw_report`.
`qeph_wall_raw_read_check` comprises `WallRawReadTest.cpp` and its small
`WallRawReadTestFixture.cpp`, linked to the reader and GTest main. Preserve strict
floating-point and single-thread Eigen flags. The five functions cover a complete
65-file synthetic collection with failed derivative checks, incomplete nonfinite
data, exact output preservation, external binding/budget/missing/extra/symlink
failures, and rehashed model/probe/contact corruption that reaches semantic checks.
Native output matrices in these protocol fixtures are explicitly synthetic;
physical contact fields come from the real host law. No native interval calls
or spectral/impact admission are part of this test suite. Proposed runtime bound:
one CPU, 1 GiB, 60 seconds, no CUDA, serialized by root.
