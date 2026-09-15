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

`vehicle_self_contact_initial_census` is the bounded actual-geometry bridge.
Its existing fixture starts one virgin owner, reads the accepted coordinates
and complete broadphase keys once, expands all level-0 facet pairs, and calls
TL's production accepted/candidate filter certificates in order: same rigid,
coordinate AABB, face, edge-cross, vertex-edge, then vertex-vertex axes. It
publishes disjoint counts and retains the first 65,536 `exact_remaining` facet
pairs, with source/category/sample hashes, in the preflighted fixed host arena
during that same pass. After the existing already-owned key/filter rerun, it
runs only those pairs through TL's shared local-task-mask builder and bounded
fixed-triangle discovery in 4,096-pair chunks on four persistent workers. The
coupon reports potential/masked/executed tasks, raw/unique features and
intersections, deterministic output hashes, typed microsecond timings and exact
tasks/s. A sub-120-second discovery is repeated once; otherwise its first chunk
is qualified against one worker. It does not construct an event arena, make an
active-use force decision, run represented-interval crossing, step physical
state, assemble force, or enable V5 runtime acceptance. CTest labels it
`coupon;real-geometry;v5-exact-sample`, serializes its GPU use, and enforces a
180-second timeout:

```sh
ctest --test-dir <build> -R '^vehicle_self_contact_initial_census$' \
  --output-on-failure -j1
```

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

The `vehicle_self_contact_real_geometry_coupon` is a separate bounded host
bridge between those tiers. The existing authenticated source wrapper checks
and extracts the pinned original member, and the existing `CanonicalSource`
and `VehicleSourcePlan` readers authenticate the complete source authority.
The focused fixture extracts only the three blocker parents and their
edge-incident neighborhood from that backing. Coordinates, source node IDs,
PID/MID/SID and section thickness are source-derived; diagnostic coordinates
or edge parameters are not copied into the test.

The small function sends EID/facets `2100084:0` versus `2279821:1` and
`2100084:0` versus `2288735:0` through typed `FixedContactFacet` evaluation and
`FixedTriangleFeatureDiscovery`. It checks the source-derived distance against
an independent long-double segment oracle, including the published
representation-error allowance, and distinguishes strict interior EE from
boundary vertex-edge EE. Same-PID disconnected parents remain discoverable.
The medium function adds only facets incident on the four discovered canonical
edges. It checks deterministic seam deduplication while retaining a separate
positive parent-local area owner for each source EID.

This coupon deliberately does not construct `VehiclePhysicalDynamics`.
Complete area/force/STI policy remains in TL's synthetic physical
`self_contact_active_uses` and `self_contact_transaction_cuda` coupons; the
source proof binds those policy tests to this real source-ID/hash extraction.
Build and run the real bridge independently of shell execution and physical
dynamics with:

```sh
cmake -S case/vehicle_self_contact -B <build> \
  -DChrono_DIR=<chrono-package> -DROBO_DYNA_TL_ROOT=<tl-source> \
  -DROBO_DYNA_VEHICLE_SELF_CONTACT_REAL_GEOMETRY=ON \
  -DROBO_DYNA_VEHICLE_CANONICAL=<canonical-directory> \
  -DROBO_DYNA_VEHICLE_SCOPE=<scope-report> \
  -DROBO_DYNA_VEHICLE_DECLARATIONS=<declaration-report>
cmake --build <build> --target \
  robo_dyna_vehicle_self_contact_real_geometry_check -j1
ctest --test-dir <build> -L real-geometry \
  --output-on-failure -j1
```

It is labelled `coupon;real-geometry`, has a 30-second timeout, and leaves the
full `acceptance-v5` suite opt-in. It does not enable original contact settings
and has no `IGNORE=1` route.
