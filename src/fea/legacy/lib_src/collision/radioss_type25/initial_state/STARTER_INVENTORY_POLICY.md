# Starter candidate policy and consumed gap phase

Pinned a62b27e6 source inspection; implementation contract, not a qualified source
producer. ININT3's IDDLEVEL1 initial search uses all expanded G mains as both NRTM
and NRTMT. The existing Engine primary-only inventory cannot be selected by just
passing TT0 or manufacturing a P count. GPU sort/range/task/count/scan/fill can be
shared; predicates and original COR3T/PEN3A packing need a distinct closed policy.

Exact current source order in starter/source/interfaces/inter3d1/inint3.F:
I25STI3/GAPM, I25NEIGH, I25INI_GAP_N, I25NORM, geometric removal, I25BUC_VOX1,
COR3/PEN3/PWR3, then later final TYPE2 removal and PREPARE_INT25. The qualified
post-GAPM app handle owns pre-BUC I25INI_GAP_N corners. Its separate masked
geometry.solid_length diagnostic must not be substituted into those corners.

I25BUC_VOX1 lines144–180 computes all four edge lengths. For MSEGTYP0 or >G it
sets GAP_N(1,L)=min(maximum edge length,previous corner1); this is the actual
GAP_NM array already normalized by I25INI_GAP_N. Initial COR3 later consumes it.
Preserve the immutable pre-BUC source handle, stage a private finalized gap array
and keep the actual changed-corner count. Carry the resulting values into Engine
source if any change occurs. No blanket assumption of change or no-op is made.

For IDDLEVEL1 the same BUC routine resolves IELEM from first solid support or
whole INSOL25 fallback. Full solid incidence gives each secondary's unique part
membership. This is distinct from a contact face's declared parent or final
mechanical shell owner; use actual solid part IDs and original reader connectivity.

Under INACTI5, BUC builds per-node maximum squared edges from **positive-STF
expanded contact faces**, not all physical solid edges. It marks nodes on solid
or coating faces among those contributors. For each STFN!=0 secondary,
EDGE_L2=HALF*sqrt(max squared edge). LEDGMAX is an ordered sum over marked NSV
entries, followed by HALF/N_SOL where N_SOL counts all unique marked nodes. It is
not a maximum. Preserve serial NSV addition order on device (one bounded scalar
fold is sufficient at startup); integer unique-node counts can reduce exactly.
Large-node classification and the exact source bounds then follow the same BUC
sequence. None of these operands may be defaulted to zero because a shell-only
coupon did not consume them.

I25TRIVOX1 uses STFN==0 only for ordinary voxel admission; it skips mains whose
IELEM_M second support is nonzero and has no main-STF sign filter. It excludes
own corners and actual pre-tied geometric removals, then actual same-solid-part
membership. For solid/coating faces it visits large nodes separately and skips
them in the ordinary voxel traversal to avoid duplication. Both routes use their
literal radius association and strict AABB/plane screens. All expanded mains and
source rows remain represented even if a pair is rejected.

Starter COR3T computes gap+margin (or max with EDGE_L2 for solid/coating), applies
DGAPLOAD/DRAD, and intersects original ICODE/512 translation bits. It does not use
Engine's1.01 inflation, velocity motion, curvature or Engine gap association.
Whole Starter PEN3A is the expected oracle for the final screen. Shared closest
point operations will be used only after expression/tie/side comparisons prove
agreement; feeding these values through Engine PackLocal is incorrect.

The new policy will count all complete required encounters/tasks/pairs before
publishing or returning an opaque seed. Source predicates have no user callback;
only the closed Engine and Starter modes can instantiate the shared sweep. Source
stamps, phase, complete scalar/solid incidence and per-main maps must validate
before launching. Limits and byte forecasts cover source upload, retained gap
output, edge/part scratch, sweep scratch, row incidence and temporary seed at the
same peak. The physical owner remains the sole clock/state publisher.
