# Explicit extended-solid physical source model

`physical_domain::Policy::RetainedShellAssembliesExtendedSolidsV4` requires the
already authenticated `solid_source::Policy::OriginalExtendedSolidsV4`: all4900
cells in16 original parts, with family counts908/1991/350/306/1345. The existing
`RetainedShellAssembliesV1` default and its2412-cell source pairing remain strict.
The factory owns no CUDA state, force history, clock, wall or runtime admission.

The original full759 plain-group/20 PART-root/155 point-card census remains.
Selection uses actual typed source incidence and canonical coordinate order.
Groups2200175–178 and2200666/667 must retain every original member, in order,
and their original node-set IDs. Partial selection of an affected group rejects
before domain decoding; it cannot inherit V1's historical restricted-group
policy. The new original point cards2409447/2409448 at NIDs2406557/2406558 must be
present. Existing point-mass parsing and coefficient production select150 cards;
there is no mass inferred from a group or unused orientation node.

`VehiclePhysicalDomain::policy()` and `VehicleType45Source::policy()` retain the
explicit policy with the immutable source backing. New TYPE45 policy
`OriginalDirectSdiType45ExtendedSolidsV4` is paired only with this domain. It
restores spherical joints2200514/2200515 through their actual original bodies;
the unchanged44 source rows yield40 required operators and four rod boundaries
2200526–2200529. Existing TYPE45 property and `VehicleJointModel` machinery is
reused. No TT0 stiffness or runtime state is created here.

Every domain map is rebuilt against the canonical ordered selected union.
`VehiclePhysicalModel` reuses the existing five-family source packer and chooses
`InitializeWithExtendedSolids`, keeping original family and source-slot order.
Its packing forecast now charges both new typed input vectors. Repeated rear
H8 slots retain their individual native M contributions; solids and point masses
supply their actual zero scalar J. All PART/plain aggregates still consume the
single complete additive ledger. Physical CIN attachment admission accepts this
sealed domain profile and retains all11165 rows plus the existing exact no-rigid
intersection checks; its source/model maps are rebuilt, never appended.

The first root source gate measured376634 domain nodes,733 complete/22
restricted/four omitted plain groups,7473 plain member occurrences and150 mass
cards. The domain owns18,078,664 bytes and the source forecast is489,506,376
bytes. That gate passed the old three full-model tests, but the V4 model rejected
the32 MiB solid cap in its lower-bound preflight before borrowed reference reads.

`physical_model::Limits::ExtendedSolids()` now explicitly supplies a64 MiB solid
construction cap, accepted only for the sealed V4 domain policy. The V1 default
and hard ceiling remain32 MiB. Overall model/source/workstation/archive limits
are unchanged. The root retry records the exact complete model startup footprint;
64 MiB is a ceiling, not an observed payload or successful-runtime claim. Tests
require32 MiB rejection, one-byte-short rejection and exact measured-cap retry.

Airbag80 cells and two structural rod parts72 beams remain outside4900. Four
rod joints, remaining restricted groups and other omitted source paths stay
explicit obligations. This profile does not assert complete vehicle closure.

## Qualification

Author: five new small host functions pass (three domain-profile and two joint-
profile controls), eight changed production syntax units and eight new/legacy
full-source test syntax units pass under one CPU/512 MiB. No actual source,
native, CUDA or GPU execution was performed in the author lane. Reports:
`crash-work/reports/vehicle-extended-author-2.json` and
`crash-work/reports/vehicle-extended-syntax-1.json`.

Use the existing physical-model fixture settings with
`-DROBO_DYNA_VEHICLE_EXTENDED_MODEL_TESTS=ON`. Build target
`robo_dyna_vehicle_extended_model_check`; CTest
`vehicle_extended_physical_model_original` contains five functions: exact
canonical domain/group census, all4900 typed source/material/slot mappings,
independent five-family nodal mass scatter and rigid membership,40 actual joint
operators, and profile/cap/late-plain failure preserving a prior handle/retry.
The domain/census function can run independently of the model construction.

The legacy `robo_dyna_vehicle_physical_model_check` target remains unchanged.
Owning small profile targets are `robo_dyna_vehicle_physical_domain_fields_check`
and `robo_dyna_vehicle_type45_fields_check`; their existing tests remain present.
The full CIN witness/original runtime tests belong to the subsequent root
integration gate and are not claimed by this host source/model slice.
