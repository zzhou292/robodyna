# Complete retained vehicle shell execution input

`VehicleShellExecution::Prepare(model)` joins the exact retained
`VehiclePhysicalModel` with the `VehicleSectionResolution` already owned by its
shell references. The result supplies immutable `model()`, `resolution()`,
`catalog()`, `failure()`, `execution()` and `physical()` getters. The physical
binding owns the final catalog/failure/role handles. The app keeps no second
material arena, coefficient ledger, clock or accepted state.

Every one of the 349645 source-ordered parents retains its original
EID/PID/MID/SID and complete native index: 324094 QEPH, 21301 T3 and 4250 QBAT.
The original 4251 midlayer cells share physical topology with both glass layers;
distinct source parents remain distinct. Existing placement and reference M/J
stay in the same shell binding and physical ledger.

The 5102 original MAT20 skins receive the explicit nonconstitutive execution
role, zero material points and None failure. Their original MAT_RIGID cards,
NIP3 section and reference E/rho/nu/thickness remain retained. TL validates their
actual prepared PART/root memberships. Constitutive shells on nodes belonging
to a rigid group keep their own source material roles. Original MID/SID 2000524
is shared by its true one-point triangle and four-point QBAT quads; no second
SID or phantom NIP3 history is introduced.

Packing copies the previously resolved rate, continuation, analytic/table and
failure declarations. Shared MID/SID values must match all named native fields,
including binary64 sign bits. Only referenced supplied hardening curves enter
the catalog, with first-source ownership and exact duplicate-ID value checks.
The small `NativeDeclarationViews` helpers are also used by the existing source
assembly adapter; its construction order, values and arithmetic are unchanged.

Preflight checks the retained original profile, complete source/reference/family
association, counts and limits before packing. It charges the physical model's
inclusive source forecast, exact app object and vector capacities, bounded
control allowances and explicit native module reservations. Shared backing is
conservatively charged across reservations. The default 12 GiB total is a
construction payload cap, not predicted RSS or an allocation request. Each TL
module enforces its own smaller cap; no count-only bypass exists. Preparation
publishes the new shared handle only after the final physical join succeeds.

Six small host functions pass under a 1 CPU / 512 MiB guard. These include a
seven-parent TL catalog/failure/PART/physical join with coincident layers, plus
typed packing, source conflicts, curve ownership, overflow and exact total-cap
checks. All 13 production/test translation units pass host syntax and the
owning CMake configuration succeeds. First author test1 failed only its expected
first-source pointer for equal duplicate curves; the implementation now retains
the first source after checking every duplicate. That failed report is retained.

Three original-source functions are authored for root execution. They validate
every parent/failure role, all original material modes/curve values, all 4251
three-layer coincidences, original triangle EID2357656, 5102 rigid skins and
merged roots, immutable lifetime, inclusive budget and late physical-cap retry.
They have not been executed by the author. No native, Fortran, GPU or whole-car
runtime claim is made by this host integration. The mandatory CIN/owner/DOF
startup, 38 retained regular joints, contact and common physical publication
remain separately owned work.

Configure this directory with the same Chrono/TL and authenticated fixture
arguments as `case/vehicle_startup/physical_model`, adding
`-DROBO_DYNA_VEHICLE_EXECUTION_ORIGINAL=ON`. Build targets are
`robo_dyna_vehicle_shell_execution_values_check`,
`robo_dyna_vehicle_shell_execution_small_check` and
`robo_dyna_vehicle_shell_execution_original_check`. Run
`ctest --test-dir BUILD --output-on-failure -R '^vehicle_shell_execution_'`.
The original source runner reuses the existing authenticated canonical fixture
reader and full physical-model test source construction.

## Self-contact test tiers

Full canonical V5 runtime startup and one-attempt tests are acceptance tests,
not unit tests. They are unregistered by default even when
`ROBO_DYNA_VEHICLE_SELF_CONTACT_SOURCE_ORIGINAL=ON`. Reconfigure an existing
parent build explicitly with:

```sh
cmake -S <same-source> -B <same-build> \
  -DROBO_DYNA_ENABLE_V5_SELF_CONTACT_ACCEPTANCE=ON
ctest --test-dir <same-build> -L acceptance-v5 --output-on-failure -j1
```

The self-only startup/attempt tests require the authenticated V5 fixture. The
wall+self counterparts additionally require
`ROBO_DYNA_VEHICLE_WALL_MANIFEST`; without it those two tests remain
unregistered. All four use the `acceptance-v5` label and retain their serialized
GPU lock and finite timeouts. No `IGNORE=1` path exists.

The standalone `case/vehicle_self_contact` build keeps normal CTest small:
`unit` covers values and source proof, while `coupon` uses synthetic forecasts
to verify the self-only budget, the combined two-slot wall+self publication
charge, and mapped-wall-before-self-contact receipt order. The factory APIs
require a complete physical model, so this host coupon does not invent a
miniature V5. Actual small runtime mechanics are qualified by TL's
`self_contact_transaction_cuda` coupon: it uses only a few physical
parents/facets and covers mixed separated/contact decisions, VF, boundary
vertex-edge, strict EE, same-body exclusion, force/STI, and candidate
rollback/retry without loading canonical V5 data.
