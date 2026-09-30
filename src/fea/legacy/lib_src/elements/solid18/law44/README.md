# Selected rear-metal LAW44 H8 reference

This immutable value admits the explicit LAW44 / Isolid18 / engine JHBE17 /
2×2×2 / ICPRE1 / ISMSTR2 / JCVT1 profile. It retains eight original source
slots. Topology is either eight distinct NIDs or the exact original six-NID
encoding `[A,B,C,D,E,E,F,F]`. Every repeated NID must have identical finite
coordinate bits, including the sign of zero. Other repeated patterns reject.

`InitializeReference` validates source/profile/topology before reusing the shared
native startup geometry body: signed center volume, initial native reversal,
frame, center and eight point Jacobians/volumes, characteristic length, SVALUE0
global density, and SMASS3 per-slot mass. Initial reversal changes only the
native-to-source permutation. Source IDs, slot order and coordinates remain
owned unchanged. The old LAW36 reference entry still rejects this profile and
all repeated-node inputs; the LAW90 wrapper also keeps its distinct-node scope.

Eight mass contributions are retained even when six NIDs are present. The
repeated source slots must later add twice to their actual common node. There
is no division by unique-node count, new geometry formula, six-node mapping,
material recurrence, force/history/model/ledger admission, or owner in this slice.
Output publication is staged; aliased input and late rejection preserve output.
The independent native qualifier observes actual repeated-node SMASS3 scatter.
