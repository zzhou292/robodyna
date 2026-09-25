# Native TYPE25 candidate staging

Pinned OpenRadioss a62b27e6, MYREAL8. This module builds a complete numerical
node/main candidate inventory. It never applies forces, owns history or advances
a physical clock. Production is C++/CUDA and has no native Fortran dependency.

The selected source scope is local roles, ILEV1, IEDGE0, IGAP1, FLAGREMNODE2 and
ICODT0..7. Actual source IDs, ordered four-slot topology, repeated secondary/main
occurrences, genuine removal CSR, main segment type and global native NRTM remain
explicit. Negative activity, remote exchange and other source profiles are not
admitted. No absent coefficient, gap or removal producer is synthesized.

TRIVOX coordinate and diagonal tests retain strict comparisons and source
association. COR3T uses base=max(DRAD,(GAP_S+GAP_M)+DGAPLOAD), pair velocity extrema
VX/VY/VZ against all four main slots, VDT=((VX+VY)+VZ)*DT1 and
GAPV=ONEP01*((base+CURV_MAX)+VDT). Global stored_motion participates only in TRIVOX.
PEN3 evaluates the native all-T3 row for T3 and native Q4 fan rows for Q4, with
native denominator floors, edge-region decisions and PENE != 0 admission.
IBC bit1 is Z, bit2 is Y, bit4 is X. ETYP/global NRTM select solid/coating symmetry.
Repeated source-node inputs require identical coordinate/velocity bits through
the shared math/ScalarBits.h helper also used by selection/geometry. Integration
qualification retains native/CUDA differential and signed-zero admission gates. STIF,
ITYP and ISKEW are neither fabricated nor consumed by this candidate stage.

Source has explicit UnitScale and Native/Si input mode. Borrowed positions,
velocities and gaps convert on each GPU load using existing unit factors; scalar
controls/domain convert once per Stage. There is no converted whole-state copy. In Si mode, all Current lengths (domain bounds,
gaps, curvature, margin, gap_load, drad and stored_motion) are metres, velocities
are metres/second and previous_dt is seconds; native mode uses declared native
working units. Report maximum_secondary_gap is always native length.
Zero stiffness skips row fields; domain-clipped secondary rows skip gaps and
velocities. COR3T reads velocities only after own-node/removal and strict screen
admission. Finite-input arithmetic overflow rejects before publication.

The conservative x-sweep derives maximum secondary gap g* from the admitted
positive-activity, domain-clipped roster. It evaluates
A*=((margin+curvature)+max((g*+main_gap)+gap_load,drad))+stored_motion in the same
rounded order as the pair radius. Finite IEEE round-nearest additions/max are
monotone, so A* >= A(pair). The same main coordinate extrema then imply every
strict native coordinate survivor lies inside the inclusive sweep envelope.
Overflow rejects. Native voxel PMAX_GAP is an enumeration overestimate; the exact
pair screen does not consume it. The sweep can overenumerate but cannot omit a
pair admitted by that screen under the declared finite domain.

Inventory reuses BoundedArena and stable CUB radix/scan patterns. Dense ranges
split into bounded256-lane tasks. Identical immutable filter inputs drive count
and fill, capacity failure publishes nothing, and no nearest-K truncation or
implicit growth exists. Canonical order is secondary-row ordinal, global main ID,
then source main occurrence; a separate device CSR retains secondary incidence.
This storage order grants no permission to reorder stateful selection or ASS0.
Removal-list sorting changes membership lookup only, not physical/response order.

Preflight queries CUB/CUDA for exact scratch requirements; it is not a GPU-free
parser operation. Pure value headers remain independently usable without CUDA.
Initialize copies source maps into a startup arena. Stage synchronously drains one
explicit borrowed stream and copies only small control packets. Every Stage
attempt, including failure, and Discard expire that instance's prior view.
IsCurrent checks that lifetime; a consuming coordinator must additionally compare
the complete QueryStamp with its authenticated source/activity/gap/geometry/attempt
and reference context. Query metadata is not physical authority. Device failure
poisons the instance; numerical/capacity rejection permits retry without publishing.

The common contact coordinator must hold two owners, leave its accepted instance
untouched while staging the other, forecast both arenas/CUB workspaces, call Stage
only when native maintenance requests an inventory rebuild, and publish reference,
inventory, history and physical state together. This module exposes no physical
Commit or standalone reference reuse receipt. Reports count this module's explicit
kernel launches, CUB calls and host fences separately; they are not profiler counts
of CUB's internal kernel launches or whole-solver timings. On errors, launch/call diagnostics
describe the attempted schedule rather than proving that every launch executed.

Qualification compares whole native PEN3/COR3T and exact native TRIVOX screen
chunks on mixed, warped, degenerate, ULP, symmetry and velocity cases. An exhaustive
complete-role native-pair oracle checks CUDA sweep membership, multiplicity,
canonical storage and CSR over updates. Other gates cover dense/exact capacities,
empty/inactive/clipped fields, native/SI views, retry/view expiry, alias rejection,
CUDA poison and production-only consumer linking. Whole native voxel scheduling,
original removal-roster generation, stateful selection/history, endpoint ASS0 and
common physical publication are separate remaining integration gates. This is
not a complete contact solver or a matched CPU/GPU performance claim.
