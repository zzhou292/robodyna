# Version4 declared TYPE2 patch

The named `tied_patch_capped.json` supplies17 real physical nodes and10 shell
parents: eight fixed-wall T3s, one10×10mm master Q4 and one off-center5×5mm
dependent Q4. Both patches start at z2mm with the same declared velocity. No
auxiliary rigid primary, point mass, prescribed transfer weights or fixed CIN
member is added. Material, shell controls and the300ns DTIX cap retain the
qualified scene's source values; actual stable timesteps must still be observed.

TYPE25 interface1 includes all ten shell parents and retains Irem_i2=1. TYPE2
interface2 declares node group3 against one actual master surface2, with
Ignore2/Spotflag28/Level0/Isearch0/Idel2=1/dsearch0 and explicit Stfac1/Visc.05/
Istf2. The2024 deck uses the original radioss2017 seven-integer card plus a blank
eighth slot. The2025 secondary-surface field is not emitted into the older card.

The source reader normalizes Isearch0 to2, keeps ILEV28 and forces hierarchy0.
These are source controls, not an observation that every row takes CIN. Actual
search, KINCHK/compaction, IRUPT0, master witness, ordered TYPE2 roster and
TYPE25 pair-removal CSR remain qualification requirements. CIN's native force
operator includes centroid/inertia terms; search bilinear coordinates are not
prescribed response weights. `tied-source-provenance.json` pins the source audit.

This change authors Python declaration/export only. The existing C++ physical
factory/runner continues rejecting unknownv4 until the genuine tied source,
owner and removal stages are integrated. No Starter/Engine invocation is part
of export. Version1/2/3 bytes remain tested unchanged. The native observation and
matched GPU17-node trajectory require separate reviewed packages and receipts.
