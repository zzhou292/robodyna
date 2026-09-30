# Native selection lifecycle oracle

Qualification-only OpenRadioss source at `a62b27e6`. Production C++/CUDA never
calls this library. Source generation and review are not numerical qualification,
a physical contact result or a whole-scene timing claim.

The reference composes the existing native history, retained/continuation/new-
impact, pair-coefficient and raw-geometry oracles. It does not call the production
lifecycle controller or its numerical functions. C++ files marshal source tables
and order native calls; the native arithmetic, membership and append predicates
come from complete original routines or exact original blocks.

## Source and compilation boundary

`prepare.py` reuses selection's pinned extraction/constant parser. It retains
complete I25OPTCD, PREP_SLID_1, PREP_SLID_2 and KEEPF. The foreign-module name is
changed to a private namespace; local wrappers admit NSPMD1/NSNR0 and leave all
foreign pointers unallocated. Private COMMON blocks carry explicit DT1, IRESP
and the serial reference schedule. Memory shims allocate the original temporary
integer arrays; barrier/register-boundary shims perform no arithmetic.

OPTCD's original local-candidate ordinal is observed at the actual output write,
and its full required count immediately before the original capacity condition.
These assignments never enter source predicates. Duplicate pairs are not matched
by value afterward. The COMP_2 membership loops, MAIN_SLID slot clear, MAIN_OPT_TRI
leave-marker release and MAINF post-force normalization are copied unchanged.
Source files and ABI shapes are pinned in `source-manifest.json`.

The reference is serial (`NTHREAD=1`, `ITASK=0`, `NVSIZ=MVSIZ=128` for OPTCD).
Previously qualified scalar classification references use their original one-row
wrappers and fold a shared row in original occurrence order. There is no threaded
reference performance claim. The original declaration/loop source is retained;
no OpenRadioss object becomes a production dependency.

## Complete ordered reference

1. Begin history once and construct the unique retained occurrence from its
   native result. The retained prefix follows secondary-row order.
2. Run complete OPTCD over the raw search roster. It flushes persistent ICONT_I
   for nonzero IRTLM1, requires the exact MG/KLEAVE/stiffness conditions and
   applies original z/y/x inclusive velocity/gap/precision screens. Observe
   admitted raw ordinals, preserve duplicate occurrences and source order, then
   release negative IRTLM3 exactly where MAIN_OPT_TRI does.
3. Clear all sliding slots, classify the retained prefix and run PREP_SLID_1.
   Q4 tagged writes preserve signed ADMSR and slot position; T3 writes only slots
   1 through 3. The native still-contact branch may update the reused metric.
4. Run complete PREP_SLID_2 with actual normal-to-main and removed-main CSR.
   Preserve source exclusions and first occurrence among newly appended mains.
5. Snapshot the entire COMP_2 continuation list, execute it in original order,
   then snapshot and execute the entire new-impact list. Preserve side-B cache
   main rewrites independently of the secondary row's global winner.
6. Run the existing native MAINF contact-loss reset before KEEPF. Its successive
   original INDEX entries are evaluated in order; this preserves earlier marker
   clearing and only admits masked cache reads when their actual branch needs
   them. Rejected candidate signs and zero-sum row clearing are observed.
7. Bind every kept occurrence to its final main, the current row sector and a
   native-defined cached barycentric channel. Resolve force K through the native
   min/clamp coefficient oracle, then call the existing raw geometry reference.
   An undefined native XP read throws and remains a composed admission failure.
8. Return rows, complete ordered occurrences and aligned raw geometry. Geometry
   offset/history, normal/friction and force assembly retain their owning phases.
   `OracleFinish` is separate and performs only the source post-force IRTLM1
   normalization; the caller invokes it after those phases, never early.

Native time and previous DT1 are supplied values, not a new clock. X_n and V
borrow the existing strided VectorView. An explicit SI boundary divides X by
native length and V by native length/time; all coefficients, gaps, histories and
TT/DT1 remain native. Node IDs and native row/main/reference identities are kept
separate from zero-based C++ array positions.

The raw inventory contains only native secondary/main identities. No incoming
cache is necessary: retained rows receive GLOB1; OPTCD admits only MG=0 rows,
which have no retained/sliding contribution and all enter the new-impact list;
sliding is derived from retained MG>0 rows and excludes that main, so every
append enters the continuation list. DST1/GLOB1/PREP1 do not change MG. The
reference independently checks that every output has exactly one completed
cache-producing phase before KEEPF or geometry consumes it. Unclassified output
is rejected; API-zero scratch never acquires a native-defined mask by default.

Unknown precision is rejected. Explicit raw IRESP0/1/2 selects the original
condition:1 uses the special floor and0/2 use ELSE/EM8. The actual scene probe3
observed0; source/receipt binding remains the coordinator's responsibility.
Branch coupons retain MYREAL8 arithmetic and do not claim whole single-precision
Engine equivalence.

## Resource and undefined-input limits

The C++ reference is bounded to 32 mains (the inherited BeginOracle limit),
128 secondaries, 256 nodes/references, 4096 raw occurrences and 8192 entries per
CSR. These are reference limits, not physical production capacities. Every
native index is checked before the original code reads it; no fabricated
subtriangle or unrequested abs repairs PREP_SLID's direct indices.

Reference composition provisions the full retained-plus-raw OPTCD bound and the
full prefix-plus-rows-times-mains sliding bound. Adequate-capacity outputs are
compared numerically. OPTCD's low-level native behavior on insufficient capacity
is explicitly different: I_OPT_STOK stays unchanged while individually fitting
output writes can still occur. Its complete required count is observed
separately. The bounded production adapter may reject atomically in private
staging; it must not call native partial publication a complete result.

Coordinator leases, source authenticity, accepted/trial publication and physical
clock ownership remain outside this package. No successful helper, matching row
or source-generation number grants physical contact authority.
