# Original retained-shell mechanical startup model

`VehiclePhysicalModel` joins the selected source producers on one immutable
physical domain. It reuses the full shell binding, original TYPE13 conversion,
TYPE25 source adapter, typed three-family solid model, point-mass mapper,
coefficient ledger and native PART/plain rigid assembly. Source packing is split
into beam, solid and plain-group units; numerical formulas remain in TL-FEA.

The prepared value retains all349645 selected shells,4442 TYPE13 connections,
2828 TYPE25 welds,2412 adhesive/rubber solids and148 main-member point masses.
All372435 domain nodes have a real coefficient producer. Native assembly
creates20 PART roots and753 selected plain groups, with12824 members, including
146 members with genuinely zero scalar rotational inertia. No guessed inertia,
curb-mass redistribution, material substitution or duplicate topology removal
is performed by this join.

This is the selected immutable coefficient/rigid startup model. It does not
admit a connected or running vehicle. The38 retained regular joints, original
CIN execution, explicit auxiliary/test-setup omissions, material/failure shell
execution catalogs, DOFs and CUDA state remain separate integration blocks.
All26 omitted non-shell PIDs and six released boundary joints require source
dispositions. The retained-source policy does not claim their mass/stiffness is
zero, nor does successful coefficient coverage establish their load paths.

The selected coefficient mass is696.25150990858265 kg, including
16.841404800000049 kg of main-member point masses. This is not full-vehicle or
curb mass. Thirty-five retained mass-card IDs also occur as different original
TYPE13 beam IDs. TL92bcfac correctly keeps the nodal-mass/ADMAS namespace
independent while rejecting duplicates within each structural or mass namespace.
The original IDs, nodes and both physical contributions remain unchanged.

Native caps include retained upstream backing: the shell map itself retains
the300 MB shell binding, and the rigid binding retains the entire ledger.
The deliberately conservative inclusive source forecast is5687373358 bytes
under a6 GiB cap; it overcharges shared backing across module reservations.
The owning source gate sampled1018769408 bytes RSS on its first successful run.
Owned payload APIs report396944640 bytes for the ledger and398239088 bytes for
the rigid binding, both including shared upstream backing. They must not be
mistaken for incremental allocations or added repeatedly as actual usage.

Three original-source functions pass in `vehicle-physical-model-root-tests-4`.
They check every mapped original endpoint/solid/source slot, complete producer
and rigid inventories, real J0, all35 mass/beam namespace intersections, exact
source coefficient sums, immutable lifetime, exact limits and a late rigid-cap
failure followed by retry. Test1 exposed the initially undersized inclusive
module caps; test2 exposed the real mass/beam identity collision. Both failures
remain recorded. Test3 passed numerically but its GTest property overload
narrowed the byte count; test4 preserves integer widths and binary64 quantity
precision in the evidence. No force/constitutive tolerance was changed.

## Rear source to immutable model

The existing private `PrepareSolids` adapter also accepts the explicit rear V3
source profile. It appends typed LAW44 inputs in their retained source order
and selects TL's extended immutable model profile. The model owns its material
curves after the source and temporary packing vectors are destroyed. All eight
source slots, including repeated rear H8 node IDs, retain their domain mapping.

`vehicle_rear_solid_model` checks all3,555 source parents, both actual rear
moduli,109 repeated mappings, curve lifetime, rejected incomplete domain and
insufficient budget, and a successful retry. The original complete physical
model tests remain enabled. Root's `rear-solid-model-bridge-tests-1` passed all
27 numerical functions across seven CTests, including the two new functions.

This adapter does not change the full-case selection: `Preflight` still requires
the existing2,412-solid inventory. A future explicit physical case profile must
compose its complete original groups, V4 coefficients, CIN and resident state
before selecting the larger source population.
