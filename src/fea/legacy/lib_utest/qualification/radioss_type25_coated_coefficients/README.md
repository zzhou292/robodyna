# Coated shell contact coefficients

The native I25GAPM block first constructs solid STF, then evaluates the ordinary
material shell STC. The first shell side keeps MAX(STF,STC); its encoded partner
receives raw STC. This value package composes those existing formulas without
admitting a source, contact owner or runtime. Only exterior EightSlot support
with resolved ordinary shell material is admitted. Null material, internal solid
faces, stacks and higher-order supports need separate source/profile work.

The native oracle extracts the complete shell/MAX/partner assignment blocks and
solid arithmetic from the already authenticated donor. Native compiler expected
values never call production math. Existing ordinary coefficient tests run with
the same refactored helper; its previous MAX behavior remains unchanged.

All operands are native working quantities. INSOL3D area, VOLINT volume, shell
membership/orientation and source units remain binding obligations; stiffness is
not a physical nodal mass, structural STI or contact gap. The returned solid
length is the intermediate GAP_N before later nodal gap distribution.

The coated support accepts finite signed nonzero reader-phase VOLINT: declared
PENTA6 can still have negative volume before INITIA. The native shell MAX then
retains shell STC when solid STF is negative. Do not use absolute volume or
reorient geometry. Ordinary exterior-solid admission remains positive-volume;
zero/nonfinite support volume rejects without publication. The actual741-row
Yaris diagnostic found36 negative PENTA values and zero primary reversals.
