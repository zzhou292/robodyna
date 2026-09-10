# Explicit layered source sections (V3)

`compile_yaris_assembly.py --material-policy layered_law1_or_law44` emits
`robo-dyna.source-assembly-inventory.v3`. It reuses the original archive/member
authentication, canonical geometry, complete selected parts, source cards,
parent mappings and explicit extraction boundary. It does not classify a
geometry inventory as runnable physics or close omitted vehicle connections.

Every material row has `material_law: layered_law1` or `layered_law44`.
LAW44 retains V2's explicit `hardening_model`, original plastic/rate fields and
optional table. LAW1 retains only `material_id`, `density_kg_m3`, `young_pa`,
`poisson_ratio`, `cards`, `source` and its law tag. There are no synthetic
plastic, rate, yield, failure or curve declarations on an elastic row.
Parents whose material has no curve retain `source_curve_id: 0` and
`curve_index: null`; a pure elastic inventory has an empty curve table.

The strict elastic card requires positive original RO/E, explicit
`0 <= PR < 0.5`, and blank optional columns. Original signed zeros, literal
cards and SI conversion results remain authenticated. Existing supported
part/section rules stay unchanged (ELFORM2/16, uniform thickness and NIP3).
The top-level `section_policy` names `layered_law1_or_law44` and pins the
LAW1 converter/section source to OpenRadioss revision
`a62b27e6baa555d222a580d6218867d0be4d70b5`; `law44_policy` retains V2's
separate hardening conversion declaration. Both remain unqualified for case
integration in source metadata.

The immutable C++ reader appends `MaterialLaw` with a legacy LAW44 default.
The source material adapter maps LAW1 to TL's `LayeredLaw1Nip3` and sets
unused controls to their canonical API defaults. These controls are not
source declarations. V3 `SourceAssemblyBindings::Prepare` explicitly uses
`InitializeSections`; `ElasticParameters`, `Parameters` and `Law` expose
typed availability. Native references, mass/J and parent order still come
from the existing single shell collection. No new state or mass owner exists.

V1/V2 keep their old material admission and initialization. V3 is host-only
in this patch: `SourceAssemblyWallSetup` rejects it before allocation and
the existing wall archive schema continues to require V1. Mixed resident
dispatch, accepted output and dynamics need their own integration gates.

The focused host fixtures select the complete original elastic steering arm
PID2000511 (135 QEPH + 14 T3), then combine it with analytic PID2000064 and
tabulated PID2000157 (581 QEPH + 50 T3 total). All three material roles occur
within the same native families. Selected internal groups and outgoing
weld/tie declarations remain explicit; this is no vehicle load-path closure
claim. Existing 1024-parent/2048-node host bounds suffice unchanged.

The original arm's outgoing groups2200007/2200670 also contain external nodes
2411580/2406582. These nodes have original point masses and spherical joints,
and no shell/solid/beam incidence. V3 alone records
`attachments.auxiliary_frontier` with policy
`released_external_auxiliary_nodes_v1`: complete original source blocks,
source-ordered mass card references, exact supplied tonne/SI mass values and
joint endpoint identities. Coverage must equal the source-verified external
nodes lacking structural incidence. Both mass and supported literal joint
evidence are required; unknown or absent evidence still rejects the inventory.
The two masses (EIDs2409487/2409471, 0.010001kg each) remain external and are
never added to selected shell mass or owner nodes. Outgoing groups remain
complete and explicitly released. This is source evidence, not joint runtime
qualification. V1/V2 keep their original structural-frontier rejection.

`robo_dyna_source_section_input_check` is owned by `SourceSectionChecks.cmake`.
It requires explicit frozen elastic/mixed paths and existing V1/V2 fixtures.
It checks complete native binding, typed availability, copy/move lifetime,
source-unit/card-bit and late parent corruption, host budget retry, and closed
wall/archive admission. The Python `test_assembly_sections` suite also checks
literal negative-zero retention and complete geometry/frontier preservation.
