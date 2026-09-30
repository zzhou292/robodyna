# Current TYPE25 normal cache and NORMP plan

Status: source-backed design, before current-normal production implementation.
The separately qualified normal_activation leaf supplies integer masks. The
existing FixedMainSource/Transaction contract remains fixed; a moving/deforming
profile must be separately named and admitted. No source clock or physical owner
is added by these numerical stages.

## Reuse and phase boundary

Root factor a75eab40 exposes lifecycle::PrepareRowBeforeNormals and
PrepareRowAfterNormals. The first performs Begin/OPTCD and the native deleted-main
release, preserving its original optimization operands; it does not read float
normal/cache geometry. A complete batch normal-update barrier belongs between
these two calls. Coefficients, X/V, controls, spatial inventory and staged history
must keep the same authenticated force-base snapshot through that barrier.

The immutable topology contains original source/parent IDs, primary/expanded
mapping, ordered node connectivity, signed MSEGTYP, ADMSR, MVOISIN/EVOISIN and
native normal/removal CSRs. Reuse startup::Main and its proven mappings; do not
infer partner IDs from arbitrary geometry or weld equal coordinates. Current
float normal slots, LBOUND and VTX_BISECTOR belong to separate accepted/trial
cache slabs. Existing static SourceView remains a compatibility wrapper; moving
views bind the current slab explicitly. A new profile cannot silently relax the
fixed-source constraint promise.

Activation inputs are existing SourceView plus actual lifecycle::OptimizedRow
history, exact optimized suffix IDs and fresh complete I25FREE_BOUND list. Root's
normal_activation leaf marks only integer0/1 ACTNOR/TAGNOD. Its complete ordered
free-list validation uses coefficient>0 and at least one true boundary edge,
omitting the repeated T3 third edge. Optimized count/domain checks do not grant
roster completeness authority; the owner binds the actual OPTCD emission.

## Exact current-field stages

Pinned i25norm.F/I25MAIN_NORM source establishes these distinct stages:

1. Begin trial from the accepted persistent face-normal cache. Clear all trial
   VTX_BISECTOR channels as MAIN_NORM does. The initial cache comes from the
   genuine Starter stage, not a fabricated all-active current update.
2. For each original primary, compute TAGE from whether all four source nodes
   have TAGNOD0. TAGE1 leaves its cached normal and partner fields untouched.
   Otherwise, positive STFM recomputes the original REAL4 primary normal; nonpositive
   STFM writes the source zero normals. Preserve exact opposite-side permutation
   and sign. T3 writes only source slots1/2/4; unused slot3 is not invented.
3. Reset LBOUND. In original ascending free-main/edge order, inspect the raw
   pre-free-edge normal. A zero raw vector gets sentinel slot3 without increment;
   otherwise each endpoint receives its exact next integer boundary count.
   Counts above2 invoke the original zero-bisector special treatment.
4. Transform every admitted free edge using DOUBLE position differences assigned
   to REAL4, then original REAL4 cross/normalization. This includes raw-zero edges.
   Store the edge direction and fill only the previously assigned slots1/2.
   Counting eligibility must be retained before this transform; a nonzero raw
   normal can produce a zero direction and still owns its original slot.
5. FLAG2 first gathers WNOD for ACTNOR-nonzero faces from the complete FLAG1
   neighbor snapshot. Nonpositive STFM gives zero WNOD. After the native barrier,
   active nonpositive faces are zeroed, and each active edge with a neighbor uses
   the original own+WNOD REAL4 normalize. Inactive faces retain their cache bits.

Factor the existing startup FloatNormals arithmetic into small shared host/device
per-face, per-edge and per-average leaves. Keep Starter's power-derived floor
separate from Engine's literal REAL4 floor; preserve float-position primary
arithmetic, double-difference free edges, signed zeros and exact operation order.
The original startup wrappers must keep all existing native and observation
qualification. Do not duplicate their geometry core or renormalize source vectors.

## Bounded GPU orchestration

The mask producer uses integer atomic OR/Exch only. Primary and partner writes
are unique under the authenticated ordinary-shell topology. There are no float
atomics. Global kernel boundaries provide the original FLAG1/FLAG2 barriers.

Boundary slots can avoid a new sorter/scheduler. The authentic normal-to-main
CSR is already in ascending main/corner order, with the repeated T3 corner
omitted. One reference thread scans its incident mains, then their edges in
source order, using the pre-transform nonzero flags. It counts endpoints and
copies the corresponding transformed edge directions into the first two slots;
if count>2 it writes the original zeros. This reproduces the global free-list
subsequence for that reference without changing floating arithmetic. It requires
the actual complete ordered CSR, not a caller assertion that an arbitrary list
is sorted. Its producer/admission must prove exactly one incidence for each
distinct main/reference pair, omitting the repeated T3 corner. Otherwise the
per-reference scan of all face edges could count the same endpoints twice.
Source topology admission remains responsible for that provenance.

Each face/edge/reference result has one writer. Keep a bounded WNOD scratch
array for the FLAG2 gather-before-write dependency, reusing existing arena/launch
patterns. A separate current-normal value module may operate on caller-owned
slabs; accepted/trial ownership and common publication stay in the existing
transaction. No per-step allocation or host geometry dump is needed.

Forecast includes two face-normal slabs (48 bytes per expanded main each), two
reference slabs (actual sizeof LBOUND+two REAL4 vectors), WNOD scratch (48 bytes
per expanded main), integer masks and pre-transform edge eligibility, plus any
required incidence/index storage. Use actual aligned sizeof-based forecasts;
these rough components are not a resource admission or chosen launch width.

## Lifecycle and initial scope

I25MAIN_FREE is called by INTTRI only when IDEL7NOK_SAV requests refresh; otherwise
its list persists. The first moving/deforming profile therefore keeps topology
and main activity membership unchanged (non-eroding), with the complete fresh
initial roster remaining valid. A later deletion/reactivation profile must bind
the real source refresh trigger and accepted/trial roster generation; it cannot
just regenerate a guessed list or admit stale subsets. The arithmetic oracle
may exercise coefficient-zero branches without claiming that lifecycle is owned.

A cache produced at force-base X_n may be committed together with physical X_n+1.
Its metadata must identify the force-base operation and accepted cache generation,
not claim every inactive slot was freshly evaluated at X_n+1. Preserve the actual
native persistent-cache semantics. Rejection/discard leaves the previous accepted
slab unchanged; all kernels drain before any borrowed view ends or common commit.
No CPU retry masks a device execution failure.

## Independent oracle and gates

Reuse the frozen complete I25FREE_BOUND/I25TAGN oracle and original complete
I25NORMP FLAG1 then FLAG2. Supply explicit prior normal/cache values and actual
current coordinates/coefficients/phase masks; no production helper manufactures
expected fields. The native wrapper must preserve the literal COMMON controls,
MVSIZ129, selected local/edge0 branches and raw source slot definitions.

Tests compare complete masks, counts, cache fields and every defined REAL4 bit
across multiple updates: no optimized rows with free boundaries, retained/removal
masks, partner activation, inactive interior caches, changed positions, rigid
rotation, warped/deforming Q4/T3, zero raw normals, boundary counts, current
coefficient-zero arithmetic, discard/retry and exact caps. An all-active control
must match the old fixed-ready producer, but cannot replace sparse-mask tests.
Hardware launch-order/width permutations must retain exact results and input
bytes. Complete moving transaction tests follow the numerical module: staged
normal failure, rejected physical step, accepted swap and force-base stamp
binding. General vehicle self-contact and matched end-to-end performance remain
later explicit gates, not consequences of a scalar normal test.
