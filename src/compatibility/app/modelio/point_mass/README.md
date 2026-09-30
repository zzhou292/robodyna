# Original point masses on the declared physical domain

`VehiclePointMassSource` reuses the authenticated original ELEMENT_MASS reader
and TL `ElementMassContributions`. It maps every original card whose NID occurs
in the supplied physical domain, preserving original EID/NID/card order and the
source-unit multiplication. Every card receives an explicit retained or outside
disposition. The native producer adds mass once and contributes exactly zero
scalar inertia; no rigid aggregate, DOF, force owner or timestep is created.

This closes a real gap in the earlier rigid-PART selection. The declared
shell/TYPE13/solid/TYPE25 baseline contains 56 original point masses: all54 PART
records plus EIDs2409343 and2409344 on retained shell nodes. The other99 cards
remain outside that domain. A separately declared superset containing every
original point-mass NID maps all155 records. That superset is a mapping test,
not an automatic decision to retain unrelated non-shell assemblies.

`physical_scope/CanonicalDomain` is shared with the TYPE25 adapter. It requires
all baseline and TYPE25 source nodes and checks every supplied NID/coordinate
against the exact canonical SI bits. Additional original nodes are allowed;
this geometric check does not supply coefficients or close connection gaps.

The forecast covers the preceding immutable source phase, retained source,
coordinate decoding, mapping scratch and the bounded TL producer. Limits default
to512 MiB inclusive source handling and32 MiB native startup. The actual owner
and final archive require their separate complete forecasts.

Tests cover duplicate-node additive cards, explicit omissions, source order,
exact mass bits/J0, reversed original domain order, the two extra shell masses,
all155 source records, exact caps and late-coordinate rejection/retry. The root
CMake/GTest gate passed4 functions in `vehicle-point-mass-root-tests-1` on the
staged source. The extraction's affected TYPE25 regression gate is required
before integration is marked complete.
