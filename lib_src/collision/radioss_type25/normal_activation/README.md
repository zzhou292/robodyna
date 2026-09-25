# Current normal activation

This module produces the selected native I25TAGN integer ACTNOR/TAGNOD union.
It runs after the lifecycle Begin/OPTCD boundary and before normal refresh. It
neither computes float normals nor certifies contact completeness or advances
mechanics. The caller owns immutable topology, actual current coefficients,
staged row histories, and the exact OPTCD suffix. Reuse the one physical owner.

The selected value profile is one local partition/processor, FLAGREMN2, no edge
or foreign rows, and a fresh complete free-main roster. Native I25FREE_BOUND
selects increasing main IDs with positive STIFM and at least one zero-neighbor
edge, excluding the repeated T3 edge. Admission checks the entire list; it does
not silently repair it. Native deletion can retain this list until a specific
MAIN_FREE refresh trigger. That lifecycle is not implemented by this fresh-list
profile, and moving runtime admission must not pretend that it is.

Retained rows mark their current main, then positive normal-reference neighbors
not in that secondary's removal CSR. Optimized occurrences mark the main and
all four nodes, plus a positive opposite-main role. Free mains mark their main,
absolute opposite and free-edge endpoints. The native checks differ between
these paths; do not replace them with one blanket activity filter. All duplicate
marks are integer set unions, so parallel writers may use integer OR/Exch without
floating-point reductions. Native arithmetic bodies are not linked in production.

The allocation-free host entry first validates every consumed span, value,
row-stage identity, count and output separation. Failure preserves both output
arrays. Success clears masks and marks exact unions. Private host/device emitters
are reusable per-row/per-occurrence work items for the CUDA transaction. GPU
orchestration must clear masks, finish complete admission, launch all contributors
and complete its barrier before NORMP. This module supplies no publication token.

Normal/bisector contents are never read here. Their retained source spans still
participate in alias rejection so a mask output cannot clobber borrowed source.
Count/domain checks on the optimized suffix do not prove its origin or inclusion;
that authority remains in the transaction which produced and retained it.

Qualification uses separately pinned native I25FREE_BOUND and I25TAGN, complete
mask equality and host/CUDA scheduling permutations, not a second handwritten
reference. A passed activation coupon is not a moving-surface normal or vehicle
self-contact qualification.
