# Opt-in ISMSTR10 world-reference Jacobian

`StartupProfile::reference_strain` defaults to `LocalGeometryOnly`, preserving
the previous reference's named local frame, positions, volume, characteristic
length, source permutation and mass values. `reference_jacobian()` is null for
that profile. `TotalLagrangian10` explicitly adds a named `ReferenceJacobian`
payload: nine native inverse entries and the world-reference volume. This is an
80-byte value payload; the in-process reference/profile sizes increase. Neither
the old 44-value qualification packet nor a persistent archive is reinterpreted.

The caller also selects `WorkingLengthUnit::Metre` or `Millimetre` (the original
Yaris units). Input positions and output coefficients remain SI. Native SJACIDP
uses `1/64 / max(DET,EM20)`, so the physical floor is
`1e-20*(working_length_in_metres^3)`. No arbitrary caller coefficient/floor is
accepted. Invalid tags reject before geometry reads; legacy local-only input
requires the canonical metre tag. Source authorization and unit selection
remain the application producer's obligations.

Pinned `SINIT3:219-244` calls SCOOR3 and SJACIDP for ISMSTR10/12, before its
separate JCVT branch calls SRCOOR3 at lines259-270. This slice admits exactly
ISMSTR10, NXREF0, NSIGI0, JCVT1, ISORTH0, no geometry-closing modification and
no initial thermal fields. SCOOR3's saved double coordinates remain in world
space. The subsequent local cyclic frame must not be used to form JAC_I.

`SJACIDP:97-107` uses sequential `a+d+b+c` and `a+d-b-c` sums. These differ from
the grouped expressions used by the existing frame helper. The small new helper
preserves those source expressions and all inverse-entry order, including the
native volume floor. Orientation follows the already qualified CHECKVOLUME_8N
permutation. All fields are staged before the sole reference publication.

The native packet retains complete SCOOR3, SJACIDP and SRCOOR3 donors, native
MVSIZ arrays, and explicit coordinate interfaces extracted from the authenticated
source declarations. It executes the actual world-to-local call chain, then the
existing complete volume/length/mass leaves. The unselected MOD_CLOSE branch
fails explicitly. The public old native ABI still returns exactly 44 fields;
the new entry appends the ten directly observed JAC_I fields. It receives raw
native coordinates and applies its own literal floor; no C++ inverse or floor
is used by the oracle.

Three new host functions pass alongside the three legacy functions. They cover
analytic inverse entries, unchanged old field bits, metre/mm floor boundaries,
invalid tags, input aliasing, late mass-overflow preservation and retry. Two new
native and two new CUDA functions are authored for root execution, including
below/at/above-floor controls in both unit systems, all 1,309 original bricks,
source permutations and old field bits. Original-mm tests use the fixture's raw
coordinates directly, with the existing bounded cross-unit roundoff allowance;
an explicit coefficient perturbation must still reject.

This closes a reference-data prerequisite for total-strain HEPH/LAW42. It does
not admit material/force history, stabilization, nodal coefficients or a vehicle
owner. Existing native/CUDA baseline results remain recorded in README.md;
the new native and CUDA gates have not been executed by the author.
