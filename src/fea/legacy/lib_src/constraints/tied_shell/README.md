# Kinematic shell patch values

This allocation-free host/CUDA module implements the selected native TYPE2
Spotflag28 force and motion operations. `PreparePatch` computes the current
centroid and seven DPARA cofactor values from four ordered master slots.
`TransferLoad` distributes a secondary force and couple to those slots;
`RecoverMotion` recovers secondary velocity, spin, acceleration and angular
acceleration from master motion. These are value operations, with no clock or
accepted-state mutation. Every rejected operation preserves its entire output.

The source is OpenRadioss `a62b27e6baa555d222a580d6218867d0be4d70b5`,
`I2FOR28_CIN` with IRODDL1/WEIGHT1, and complete `I2VIROT3`.
The independent source oracle and exact extraction manifest live in
`lib_utest/qualification/tied_shell_patch/native`. Native quarter weighting and
cofactor expression order are retained; this is not a bilinear interpolation.

A triangle uses its third physical node twice, in slots three and four. Both
slots must receive the same node motion. The later nodal assembler must add both
force increments to that one physical node. Repeated coordinates alone do not
establish this identity.

The geometry guard requires a finite positive determinant greater than
`64 * DBL_EPSILON * max_diagonal^3`. This is a declared conditioning limit, not
a native clamp. Collinear, coincident, nonfinite or sufficiently ill-conditioned
patches are rejected. Current coordinates must be supplied for each evaluation.

`TransferCoefficients` stages the selected native mass/inertia/stiffness
increments. Its four `initial_master_inertia` inputs are native MINER, distinct
from the current assembled IN values. Positive inertia at all four master slots
selects inertia transfer; otherwise native offset inertia contributes additional
numerical mass. `source_mass` and `source_inertia` retain the original secondary
coefficients, while `numerical_mass_delta` reports DMAST separately. Dependent
M/J become zero and dependent stiffness becomes exact native EM20. Zero source
mass or inertia is valid; negative/nonfinite input and computed overflow reject.
These are staged slot increments, not an assembled nodal update. True repeated
triangle slots must receive identical initial inertia and assemble to one node.

Interface search/classification, source identities, coefficient assembly and
dependent-DOF participation, retained release history and owner-stage integration
remain separate work. In particular, these values do not permit an independently
integrated secondary node or duplicate nodal mass. Do not substitute runtime
MINER selection for the distinct starter I2_DTN caller. No source weld is admitted
by this module alone.


`TiedSearch.h` adds the independently qualified native candidate projection and
ordered selection. It keeps explicit working units, true repeated-third T3
semantics and native distance ties. It does not assign original-source packing
or attach the force patch. See `lib_utest/qualification/tied_shell_search/README.md`
for the eight host/native/CUDA functions and finite-overflow review corrections.
