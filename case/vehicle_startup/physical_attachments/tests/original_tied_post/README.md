# Original tied-post extraction qualification

Two small functions reuse existing factories. FinalizeOriginalTiedSearch owns
packing -> geometry -> assessment -> finalization; PrepareOriginalTiedPost owns
classification -> PostKinChk. Callers retain the auxiliary/context factories,
wall replacement policies, reader positions and error order. Wall byte temporaries
still end at the Context::Prepare statement. Finalized retains assessment and
geometry; the resulting post handle retains classification, finalized and context.
No borrowed byte, geometry or context lifetime escapes unowned.

The helper .cpp is added to the existing robo_dyna_tied_search_post_kinchk target,
which all old callers already needed. Its product link dependencies remain exactly
unchanged; no GTest, fixture, VehicleRun or OriginalCase dependency is added.

verify_sources.py authenticates both complete callers after explicit reversal,
compares the moved expressions against the original code and pins unchanged lower
producers/forecasts. The predecessor reader proof calls this reversal before
checking its original178 baseline; no frozen baseline was refreshed. The original
physical-attachment late-cap/retry test is pinned unchanged for later owning gates.

The focused CMake project reuses the predecessor's five reader tests and source
proof, plus the existing two classifier and three PostKinChk value/rejection tests:
10 GTests and4 CTests total, no new shadow implementation. A compile-only object
target checks the real helpers, OriginalYaris.cpp and fixture-facing signatures.
These checks do not invoke full source preparation, CUDA search or a physical owner.
The CUDA toolkit supplies existing public header types only.

Compilation and execution are pending. Full production link and original-source
search/attachment acceptance remain separate owning gates; do not infer them from
source proof or these small value tests. Predecessor2776 is source-reviewed, not
qualified or promoted. This branch does not implement a native CLI or full builder.
