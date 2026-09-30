# Standalone bounded CUDA facet filters

This source-only slice starts at TL `21941d20`. It is not wired into accepted or
candidate transactions or the app. No vehicle speedup or new acceptance is claimed.

`Arithmetic.h` and `Prism.h` are the existing public filter implementation,
extracted as host/device templates over a triangle's coordinate array. There is
one geometry core. The original CPU wrappers and public header are unchanged.
`verify_extraction.py` pins twenty normalized function bodies to that qualified
source and permits only the documented type/platform spellings. In particular,
unconditional `nextafter` padding, source validation, axis order and stopping,
coincident-hull rejection, signed-zero min/max ties, and invalid/overflow outcomes
are preserved. Existing Q4 interval helpers intentionally are not substituted:
their exact-zero and error-free sum rules give different enclosures.

`Batch` retains one CUDA allocation on a caller-owned explicit stream. It borrows
host scene/paired-index arrays only until synchronous upload/query return. Queries
have no explicit allocation and return one result per pair in the caller's order;
there is no compaction, sorting, nearest-K policy, physical authority or floating
atomic. A query validates all ordinals in order before launch. Nonfinite geometry
and invalid thickness are per-pair predicate outcomes, not an incomplete batch.
All borrowed reads are drained even if a later enqueue fails. CUDA failures poison
the owner and never publish partial results. Calls on the same owner and mutations
of borrowed inputs must be externally serialized. Every invocation revokes its
previous result; every Upload invocation additionally revokes its previous scene.
Invalid queries can retry against the retained scene, invalid Upload requires a new
valid upload, and successful uploads increment a descriptive scene generation.

Device layout is computed by `BoundedArenaLayout`: two `TriangleGeometry[F]`
endpoint buffers, `FacetProperties[F]`, `FixedTrianglePair[P]`, `PairResult[P]`,
with all alignment included. On the intended binary64 ABI this is 160*F+12*P
bytes (the properties include their alignment padding); qualification checks
actual `sizeof` and byte boundaries. Host retained storage is one `PairResult[P]`
plus the actual Batch/Impl owner sizes; startup additionally charges two Layout
values and a Preflight. There is no hidden second pair/result device buffer.
Borrowed caller geometry and future transaction packing storage are excluded and
must be charged by their owner. CUDA allocator overhead remains under the process
and device guards. The default F=P=4096 device payload is 704,512 bytes on that ABI.
No full-model key/descriptor structures are uploaded.

CPU compilation disables fast math and contraction. CUDA disables contraction and
FTZ and enables precise division/sqrt. The batch declines unsupported host floating
environments (non-RN, FTZ/DAZ or unmasked traps) instead of silently changing the
CPU contract. The current environment check is audited for x86; other host ABIs
return UnsupportedEnvironment until their FTZ/trap state is supported. This is a
standalone capability failure, not a new physical profile or fallback authority.

Three host groups cover compact/public parity and exact capacity accounting;
six CUDA groups cover all axis families, permutations, repeated/partial/empty
batches, overflow/subnormal/ULP/touch/degenerate/malformed geometry, earliest bad
ordinal, alias/cap/stream rejection, scene/result revocation and reuse, and floating
environment restoration. The shared corpus is deterministic and synthetic. A
benchmark in `benchmark/` is separately owned and must time complete transfers and
verification separately, identifying any optional fixture provenance explicitly.

Root owns execution: CMake targets `self_contact_filter_batch_values` and
`self_contact_filter_batch_cuda`, CTests with matching names plus
`self_contact_filter_batch_source`; Bazel host_check/source_check in this directory.
Then requalify the existing transaction certificate/source targets because they
consume the extracted host wrappers. CPU/GPU status/category/valid/axis results
must match before any transaction wiring. Promote only after a controlled actual
batch measurement including scene refresh and transfer cost. Existing acceptance
uses the unchanged integration branch while this isolated source is developed.
