# Immutable complete solid mechanics model

`solids::Model` retains three typed family spans for the already prepared
Solid18, HEPH24 and S6Z value APIs. Each parent owns its complete reference,
source-derived material association and original-slot map into one immutable
`NodalNodeDomain`. S6Z retains its explicit force profile; HEPH requires its
TotalLagrangian10 reference payload. Scalar material tables retain original MID.
There is no source parser, source-coverage receipt, force cache, clock, inverse
coefficient initialization, contact or nodal-owner admission in this model.

Inputs are per-parent `Input18`, `Input24` and `Input6z` views. An absent family
must have `{nullptr,0}`. The model requires at least one parent. Parent order is
family18, family24, family6z and then the retained order within each family.
Output parent spans preserve that order and contain `material_index` plus eight
global-node slots; S6Z uses six and has canonical SIZE_MAX in the unused tail.

Material catalogs are deduplicated by original MID, preserving first occurrence.
HEPH and S6Z share the LAW42 catalog. The first LAW36 occurrence owns a deep copy
of both curve arrays in a single bounded pool. Repeated MIDs require every named
scalar and curve value to match exactly, including signed zero; one MID cannot
identify both LAW36 and LAW42. Pointer identity and C++ padding are excluded.
All borrowed curves and input/domain handles may be destroyed after successful
initialization. Copies share immutable backing and allocate nothing.

The model derives one `SolidNodeContributions` through its existing typed API,
so global source identity, represented SI coordinate bits, original-slot masses
and EID uniqueness retain the same authority. The model's `Matches` additionally
checks material and mechanics profile identity. Equal coefficient snapshots
alone are insufficient: different stabilization or reference-strain profiles can
have equal masses. The model does not derive physical coefficients from forces,
fill uncovered domain rows or invent scalar rotational inertia.

Initialization stages all output. Counts, caps, pointer ranges and a complete
lower-bound payload check precede borrowed parent reads; curve counts and ranges
are checked before curve values. It then sizes the deduplicated pool and delegates
coefficient construction under the remaining simultaneous startup budget. Only
when material, source, profile and all parent checks succeed does the handle
publish. Failure preserves an empty handle; a prepared handle rejects reinit.

The retained payload counts the model handle/implementation, shared-control
reserve, one typed arena and the complete coefficient/domain payload minus its
already embedded handle. Startup additionally reserves typed reference staging,
material-ID/index scratch, temporary handles and the coefficient owner's own
startup scratch. These are conservative simultaneous payload reservations;
allocator-internal overhead is outside the existing utility's payload contract.
There is no private implementation-size compatibility promise or serialized ABI.

`qualification/solid_model` owns seven small host tests and a separately enabled
all-original test for 908 + 1309 + 195 parents. That test uses existing immutable
qualification fixtures; production consumes only supplied prepared references
and materials. It compares all named reference values, source IDs/slots, global
maps, full HEPH Jacobian and owned original LAW36 curve points. It does not replay
force histories or claim native/global-owner integration. Native family value
qualification and production source admission remain their existing owners.

Root-owned full-count commands:

```sh
cmake -S lib_utest/qualification/solid_model -B build/solid-model \
  -DCMAKE_BUILD_TYPE=Release -DTL_SOLID_MODEL_SOURCE=ON
cmake --build build/solid-model --target solid_model_host solid_model_source -j1
ctest --test-dir build/solid-model --output-on-failure -j1
```

Owning targets: `tl_solid_model`, `//lib_src/elements/solids:model`, and
`//lib_utest/qualification/solid_model:host_check`. Source/native/GPU jobs follow
the workspace's scheduled bounded execution policy.
