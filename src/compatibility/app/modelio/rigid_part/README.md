# Original rigid shell source profile

`RigidPartSource` authenticates the original member against the retained
canonical/source identity and prepares literal source topology. It retains 22
MAT_RIGID PARTs, 20 extra-node sets and two original body merges, producing 20
roots through the qualified TL `NodalRigidPartTopology`. No reference report is
used as production input and no new source sidecar is generated.

`VehicleSectionResolution::ResolveOriginalRigidParts` extends the explicit
original midlayer profile. Original MAT_RIGID cards remain visible; the named
converter role supplies LAW1 rho/E/nu and ELFORM2/NIP3 thickness to existing Q/T
reference startup. `SourceShellRole::OriginalRigidPart` and `rigid_root_index`
are separate from those material/formulation values. All 349,645 source rows
remain. The prior 344,543 reference values are unchanged; complete native family
indices are regenerated in source order and never substituted for source
indices or successful-reference storage indices.

Original blanks and explicit zeros remain distinguishable. Source CMO/axes
cards must remain blank. Unsupported PART/section options, inconsistent
thickness, NIP, identities or extra/merge operators reject. All raw selected
source blocks are hash checked and kept in line order. Existing bounded source
helpers decode cards and node lists; there is no second keyword parser.

PART memberships are ascending original NID coverage sets. Extra-node lists
retain their literal list order. This is a source ordering contract, not proof
of native SDI traversal/internal-node registration. No centroid is computed:
converter first-seen element traversal and starter member order remain separate
inputs for the later native raw M/J stage. No generated native IDs are invented.
The complete 759 main-member nodal-rigid groups supply the disjointness check;
this does not imply absence of auxiliary or other constraint interactions.

`root_for_node` maps an original member NID to its final source root. It uses
the validated complete membership and returns `SIZE_MAX` outside it; this is
not an owner index or native registration table.

The source retains joint, point-mass and discrete-element block evidence but
makes no connection-force, coefficient, suppression, timestep or owner claim.
The 54 extra nodes without structural incidence must still receive their actual
native coefficient/DOF treatment. In particular ELEMENT_MASS adds mass, not J.
The 22 original primary regularizers must survive the two raw tensor merges;
only 20 final roots receive principal correction. Existing raw assembly and
runtime qualification remain separate from source/reference availability.

`AdditionalForecast` includes the caller's original member bytes, metadata DOM,
raw/card payload, decoded canonical arrays, topology and temporary indexes.
The overlay charges its retained base exactly once and adds its complete native
map. Reference rows account for the new role/root fields through sizeof.
Allocator bookkeeping and process RSS are outside this existing payload-budget
convention. Old source/profile factories retain behavior and schema; there is
no new vehicle archive or live publisher in this increment.

Author qualification: seven new source/reference functions and sixteen existing
functions pass under one CPU / 512 MiB. The two complete-source/root functions
and one all-prior-reference function compile for root's separately scheduled
gate; author compilation is not complete-source or runtime admission.

Root qualification on2026-09-11 passes all26 source/reference functions,
including the three complete original-source checks. All349,645 references
succeed with zero rejection/unresolved rows; every prior344,543 named reference
field remains identical. Source forecast522,032,110 B and reference
forecast789,592,619 B fit the unchanged512/768 MiB profiles. The full reference
parity check retains both assessments and sampled784,822,272 B process RSS
under2 CPUs/3 GiB. Evidence: `crash-work/reports/vehicle-rigid-root-*`.
