# Native TYPE25 candidate inventory

Implementation boundary, 2026-09-25. Pinned OpenRadioss a62b27e6.
This stage computes search inventory, never force, penetration history or a clock.
Production reimplements native numerics in C++/CUDA; native Fortran stays in tests.

Selected scope is local node/main roles, ILEV1, IEDGE0, IGAP1, FLAGREMNODE2,
finite nonnegative native gaps/activity, explicit source symmetry controls.
Source IDs and repeated secondary/main occurrences are retained, with their
original ordinals. Missing removal/gap producers must not be filled with invented
values. Unsupported profiles fail before publishing any trial inventory.

The filter preserves strict TRIVOX coordinate/diagonal screens, COR3T packing,
native PEN3 floors/expression order/symmetry and PENE != 0 admission. T3 uses the
all-T3 row expression; Q4 uses the Q4 expression. The 2064-case native packing
experiment motivates this policy; host/CUDA differential tests remain mandatory.

Conservative sweep proof: let g* be the maximum native secondary gap among the
positive-activity, search-domain-clipped roster. Compute A* in the same rounded
order as the per-pair radius: ((margin+curvature)+max((g*+main_gap)+gap_load,drad))
+stored_motion. Finite IEEE round-nearest additions and max are monotone, so
A* >= A(pair). The same main coordinate extrema and subtraction/addition imply
every native strict-coordinate survivor lies inside the inclusive sweep envelope.
Overflow is rejection. The voxel-only PMAX_GAP overestimate does not tighten the
native exact pair screen. A sorted x sweep may therefore overenumerate; it cannot
omit any pair admitted by that screen. It does not replace native domain clipping,
own-node/removal rules, diagonal/PEN3 tests or required stateful selection order.

Retained GPU implementation will reuse BoundedArena and stable CUB radix/scan
patterns, split dense main ranges into bounded tasks, use identical immutable
filter inputs for count/fill, reject exact capacity overflow and never truncate.
Records are canonically stored by secondary source-row ordinal, global main ID,
main occurrence. This storage order does not authorize a stateful selection fold.
Reference, inventory, history and physical state are committed only together by
the existing common physical owner; standalone inventory success is not a receipt.

Qualification gate: original native PEN3 and COR3T (independent test wrappers),
mixed T3/Q4 including warped/degenerate/ULP/symmetry; independent native TRIVOX
membership on bounded scenes; complete GPU count/fill/sort/secondary incidence;
empty/dense/exact capacity, exclusions/duplicates, current gap changes, immutable
reference generations, failure/retry and actual CUDA execution. No component speed
or whole-contact claim before those gates and coupled selection/response closure.

COR3T closure: local IGAP1 uses base=max(DRAD,(GAP_S+GAP_M)+DGAPLOAD),
pair velocity extents VX/VY/VZ against all four main slots, VDT=((VX+VY)+VZ)*DT1,
then GAPV=ONEP01*((base+CURV_MAX)+VDT). Global stored_motion is only a TRIVOX
screen operand. ICODT is explicitly admitted only in0..7; its common constrained
axes produce IBC. ETYP and full source NRTM remain distinct operands. ISKEW,
STIF and ITYP are not consumed/fabricated. Tests use the whole pinned COR3T and
all32,768 admitted five-node ICODT combinations plus moving mixed-topology rows.

The implemented Inventory is a numerical staging owner, not a reference receipt.
Every Stage (including input, resource or device failure) expires its old view;
Discard expires it too. Stage completes synchronously on one explicit borrowed
stream and transfers only three small control packets. Its reusable startup arena
holds a sorted secondary sweep, bounded256-lane tasks, count/scan/fill buffers,
canonical pairs and device secondary CSR. Sorting removal lists changes membership
lookup order only; secondary/main occurrence identity and multiplicity remain.
The future common coordinator retains two owners and binds reference/history/force
publication atomically. It calls Stage only on a required native inventory rebuild.
It must forecast both arenas, validate IsCurrent before consumption, and preserve
the accepted instance while the other stages. Device failure poisons the instance;
resource/numerical errors permit retry with new current inputs and no publication.
