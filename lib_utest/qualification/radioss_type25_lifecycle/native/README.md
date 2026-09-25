# Native sliding and contact-row lifecycle oracle

This is qualification-only OpenRadioss source at `a62b27e6`. The production
C++/CUDA row pipeline does not call it. It is being composed with the existing
native history, classification and selected-geometry references; source
preparation alone is not numerical qualification or a physical contact result.

`prepare.py` reuses the established selection extraction and native constant
parser. It copies the complete `I25PREP_SLID_1`, `I25PREP_SLID_2` and `I25KEEPF`
routines, including their original local and foreign branches. The only routine
substitution is the foreign-module name. Local wrappers will admit `NSNR=0` and
bounded local indices; foreign arrays remain unallocated. `LifecycleMemory.F90`
allocates the original temporary integer arrays and contains no selection math.
Original files and reused parent donors are pinned in `source-manifest.json`.

`LifecycleMembership.F` embeds both complete `I25COMP_2` eligibility loops without
changing their predicates. It represents the serial original occurrence order
(`JTASK=NTHREAD=1`). The continuation list is a complete snapshot before any
continuation evaluation; the new-impact list is another complete snapshot after
that phase. The output tail is an explicit API zero, not a native observation.
The source-shaped view owns integer copies only for this bounded reference call.

The composed caller must preserve these source phase boundaries:

1. Begin history once and construct the retained occurrence from that result.
2. Clear all four sliding slots as `I25MAIN_SLID` does, then classify the retained
   cohort and run PREP_SLID_1. Its Q4 tagged slots preserve signed ADMSR values;
   its T3 branch writes only slots 1 through 3. No compacting or implicit abs.
3. PREP_SLID_2 consumes authentic normal-to-main and removed-main incidence.
   It counts the complete append before publication and retains the original
   candidate prefix on insufficient capacity. Its raw NOR indices must already
   be in the declared positive native reference domain.
4. Execute the two snapshotted classification lists in occurrence order. A local
   new-impact side-B winner can rewrite the occurrence main/cache even if the
   secondary row's global winner does not change.
5. Apply the existing native contact-loss reset, then KEEPF over its positive
   candidate index list. KEEPF can clear a matching row with zero cached sum and
   marks rejected occurrences by negating CAND_N.
6. Bind selected geometry to the final source main and a native-defined cached
   barycentric channel. Force stiffness is a separately resolved coefficient,
   never the classification activity product. Preserve logical geometry/history
   cohorts and explicit post-force marker normalization.

A mathematically valid source index is required before native execution. In
particular PREP_SLID_1's Q4 old-sector access and PREP_SLID_2's direct NOR access
must not be made safe by manufacturing a sector or taking an unrequested abs.
Undefined native scratch remains masked; fixture seeds test preservation only.
The reference has private COMMON state and is serial qualification code.

Coordinator leases, source authenticity, accepted/trial publication and the
physical clock remain outside this arithmetic/reference package. No new
physical authority is granted by a matching row or a successful helper call.
