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
Each focused fixture extracts only its required parents (or the three blocker
parents' edge-incident neighborhood) from that backing. Coordinates, source
node IDs, PID/MID/SID and section thickness are source-derived; diagnostic
coordinates or edge parameters are not copied into the test.

The small function sends EID/facets `2100084:0` versus `2279821:1` and
`2100084:0` versus `2288735:0` through typed `FixedContactFacet` evaluation and
`FixedTriangleFeatureDiscovery`. It checks the source-derived distance against
an independent long-double segment oracle, including the published
representation-error allowance, and distinguishes strict interior EE from
boundary vertex-edge EE. Same-PID disconnected parents remain discoverable.
The medium function adds only facets incident on the four discovered canonical
edges. It checks deterministic seam deduplication while retaining a separate
positive parent-local area owner for each source EID.

The same coupon also qualifies the first nonlocal intersection reported by the
262,144-pair accepted-V5 exact sample: EID/facets `2125365:0` and `2348922:0`,
both source PID `2000546`. The authenticated Q4 source-node rows are
`2120447,2352113,2352112,2352118` and
`2352127,2352111,2352112,2352113`; the two selected facets share only canonical
vertex `2352112`. Production local masking is exactly `0x6c30`: the two
shared-vertex VF tasks and four edge pairs incident through that vertex are
masked, while nine nonlocal tasks execute.

Production exact intersection classification is `Transverse` with
`local_exclusion=SharedVertexOnly`, so it does not require later admission. An
independent long-double plane-cut oracle finds a degenerate intersection
segment: both endpoints are the source-derived position of vertex `2352112`;
the two plane normals are nonparallel (normalized cross magnitude
`0.999999999935844330616`). Production's exact tangent-cone check proves that
the complete intersection set is that canonical shared vertex; any
positive-length departure remains fail-closed.

The nine unmasked candidates are three VF-to-edge, one VF-to-face, two strict
interior EE and three boundary EE records. All representation errors and all
zero-distance counts are zero. Distances range from
`0.010458125808179189 m` to `0.016102177762525868 m`, above the authenticated
`0.0025 m` sum of reference half-thicknesses, so there is no positive-distance
thickness contact and no zero-distance nonlocal EE (parallel or nonparallel).
Each record has a recognized VF/strict/boundary geometry class, but this coupon
does not construct active-use authority and therefore does not claim an event
admission. Swapped pair order and a permuted triangle catalog reproduce
deterministic hash `8971061864684255070`.

The coupon additionally qualifies the accepted-stage failure exposed by the
8,000,000-entry census at pair index 1409: EID/facets `2382006:0` and
`2382159:0`, both PID/MID/SID `2000893`. Their authenticated Q4 node rows are
`2402419,2402384,2402383,2402420` and
`2402419,2402383,2402385,2402534`. The selected facets share canonical
vertices `2402419` and `2402383`. Their exact common segment is the first
parent's fixed-facet triangulation diagonal but the second parent's physical
boundary, so their `FacetEdgeKey` values intentionally differ despite equal
canonical endpoint keys.

Production now recognizes that endpoint topology as
`local_exclusion=SharedEdgeOnly` only after exact intersection classification
has ruled out positive area. Coordinate-only coincident segments remain
nonlocal. The exact mask is `0x775b`; all four unmasked features are separated,
with minimum distance `0.0060364117806370014 m` above the authenticated
`0.00165 m` thickness distance. The independent plane-cut oracle reproduces
only the shared `0.018139717452543137 m` segment with zero cut mismatch.
Swapped pair order and triangle-catalog permutation reproduce deterministic
hash `2184554811619657670`.

The separate `vehicle_self_contact_candidate_rigid_coupon` reproduces the
next full-V5 candidate failure without constructing or running the V5
self-contact transaction. It advances the authenticated real-Yaris physical
owner once at the preserved `200 ns` step, copies the owner-authenticated
accepted/prepared nodal and rigid-group snapshots, and inspects only
EID/facets `2100005:0` and `2100048:0`. Their active parent/facet-use ordinals
are `4/8` and `47/94`; both are Q4 rows under PID/MID/SID `2000403`, with
`0.00086 m` reference half-thickness each. The first facet is
`LinearNodalV1`; the second is `PartialOrMixedRigid` because source node
`2269331` (domain node `157735`) belongs to binding group `612`, nodal-group
source/set `2200795`, while its other contributing nodes are ordinary.

The exact facets share only canonical vertex `2159583`. Their local task mask
is `0x0da4`, and exact accepted and prepared intersection classification
retains the existing local exclusion at both endpoints. Every unmasked
prepared closest feature remains separated: the minimum is
`0.012724253647612101 m`, versus `0.00172 m` combined contact thickness.
Their outward quadratic swept boxes overlap only because they both contain
the shared vertex. The owner trajectory is
`EndpointCorrectedSecondOrderDriftV1`,
`x(u)=(1-u)x0+u*x1-.5*u(1-u)h^2 q`, but this contributing rigid node has
prepared `omega=(0,0,0)` and therefore exact `q=omega x
(omega x (x0-c0))=(0,0,0)`. The coupon reports every parent, vertex, edge,
node/group identity, endpoint coordinate, group snapshot, and swept box; it
does not infer a linear chord from the facet-level mixed-motion label.

This proves the existing exact local-incidence policy is the legal resolution
for this pair. TL applies that prepared-coordinate policy before reporting
unsupported nonlinear motion. A nonlinear pair with no exact local
intersection, with coordinate-only overlap, or with any nonlocal intersection
continues to fail closed as `UnsupportedMotion`; work exhaustion is not
converted to a local exclusion.

The same authenticated coupon separately reproduces immediately adjacent
pair 143, `2100005:0` versus `2100048:1`. Its second facet retains the
conservative `PartialOrMixedRigid` source label and overlapping swept box, but
all three represented vertices pass TL's exact composed-curvature certificate.
For each represented point it evaluates
`sum_j weight_j*(omega_j x (omega_j x (x0_j-c0_j)))` as fixed-capacity exact
dyadic integers. Both facet trajectories are therefore affine, so
`ClassifyCandidatePairMotion` selects the existing exact linear
prism/discovery/represented-crossing path. The result follows from the
authenticated polynomial and zero prepared spin, not EID matching, a
tolerance, or endpoint-chord substitution.

For receipt pair 2694, the coupon also isolates source facets `2100124:0`
and `2209533:0`. They have no shared canonical vertex and no local task mask;
accepted/prepared endpoint discovery has no intersection and reports
`0.010390410835049144 m` minimum distance against `0.00353 m` combined
thickness. Their conservative swept boxes nevertheless overlap. The first
active facet use is transaction facet `222`; the second is `138371`, with
domain node `199834` in rigid binding group `154`.

The small physical-only preparation deliberately does not run the accepted
self-contact transaction. It therefore prints an explicit
`accepted_self_contact_assembly_included=0` baseline: group 154 has zero
curvature there, unlike the full receipt's post-assembly prepared owner.
This prevents the source coupon from falsely presenting pre-contact
coefficients as the authenticated failing candidate. TL's production
diagnostics now publish nonlinear work/depth exhaustion from the actual
prepared owner, while the general exact-dyadic/subdivision tests cover the
nonzero-curvature certificate without rerunning full V5.

`NonlinearAmbiguousFixture.bin` freezes the complete 826-pair nonlinear
geometry-result class after that source-derived pass: all post-pass possible
curved crossings plus every local-intersection/persistent-ledger pair whose
quadratic coverage must exclude a nonlocal intersection. It stores canonical
triangle/topology identities,
accepted/prepared binary64 geometry, outward exact-dyadic quadratic
coefficient intervals, both half-thicknesses, endpoint feature candidates and
representation errors, local masks/intersections, and every matching accepted
VF/EE owner certificate. Schema v2 also freezes each accepted feature's
active-use status, endpoint support, force-area values, distance decision,
canonical ledger search counts, and explicit accepted/prepared phase identity.
Its schema caps the payload at 64 MiB; the current file is 12,043,248 bytes
including its 208-byte header and has SHA-256
`9f9cc331d07ee0dd4b12a699ecda73d00a3333fdc5788aa38e4fa79415e04545`.
The pinned roster/payload/source/schema/profile/dt hashes are verified before
any proof is run. The direct `vehicle_self_contact_nonlinear_fixture` test
regenerates both endpoint feature publications, then reruns bounded quadratic
coverage with pair, ledger, and repeated-call permutations in about 3.1
seconds without loading the vehicle.

The accepted-policy evidence resolves the reported owner contradiction.
Of 826 pairs, 312 have only exact same-rigid-group close features and 514
resolve through authenticated accepted-ledger coverage. The final endpoint-root
case, `2278266:0`/`2278271:0`, is proved by a strict through-vertex separating
axis over the complete quadratic cell; the axis places both moving arms of
each triangle on opposite open half-spaces without a tolerance.
`2112794:0`/`2113455:1` is an exact
shared-vertex-only intersection. Its pair-local frozen owners select accepted
source order 9421; replay of all authenticated policy matches restores the
lower exact-pair source order 51, which is the canonical owner. Its prior
zero-thickness SAT bound remained inconclusive on `[0,2^-20]` (physical time
`[0,1.9073486328125e-13] s`) because only shared-edge topology had a
whole-cell proof. The degree-six directed-Bernstein VF/EE no-root certificate
now proves that no nonlocal feature can cross while retaining the exact local
vertex path, and separately reports the canonical pair intersection at exact
time `u=0/2^0`. No true nonexcluded accepted contact remains ownerless, and no
tolerance, seam broadening, timestep change, or thickness inflation is used.

The same authenticated no-force pass now censuses all 5,809,241 affine pairs:
926,944 swept-bound separations, 1,362,216 prism separations, 60,841 residual
separations, 25,705 persistent accepted owners, 53 represented separations,
3,433,481 represented crossings, and one represented `WorkExhausted` result.
There are no represented degeneracies or arithmetic-range failures. The sole
pair is `2142381:1`/`2230072:1`; the complete roster digest is
`6473154677596308446`.

`LinearWorkExhaustedFixture.bin` freezes that pair's endpoint paths,
half-thicknesses, zero affine quadratic coefficients, masks, features,
intersections, three candidate accepted owners, and feature-specific exclusion
evidence. It is 21,408 bytes including its 208-byte header and has SHA-256
`4ffca5180edd06640c64ca7a8ec2c84b9b460dda174f845984d77e4d0ae15236`.
Replay takes about 0.46 seconds. The pair has no common translation, residual
separation, persistent exact-pair owner, shared topology, endpoint
intersection, exact exclusion, degeneracy, or arithmetic-range failure. One
canonical seam VF owner, accepted source order 931 from
`2142378:0`/`2230072:1`, maps onto the candidate's identical source vertex and
target face; the other two candidate ledger rows do not map. Exact arithmetic
shows that `u=10067/131072` is still inside this owner's closed contact set, so
the old repeated cell was an outward-interval stall rather than an ownership
change.

The closed-set certificate proves over the complete affine interval that the
target face stays nondegenerate, the moving closest point stays strictly in
the same face interior, and the source vertex remains the unique support
vertex on one strict side of the face. Its exact degree-six clearance
polynomial is strictly decreasing with one contact-ending root isolated in
`[345915413587096,345915413587097]/2^52`. Before and at the root the canonical
VF is owned by source order 931; after it, the same face normal strictly
separates the thickened triangles. Zero-thickness geometry is strictly
separated throughout. The fixture now resolves in one work item with no
tolerance or sampled closure assumption.

The source-only real-geometry coupon deliberately does not construct
`VehiclePhysicalDynamics`.
Complete area/force/STI policy remains in TL's synthetic physical
`self_contact_active_uses` and `self_contact_transaction_cuda` coupons; the
source proof binds those policy tests to this real source-ID/hash extraction.
The transaction still rejects every `RequiresIntersectionAdmission` record
before processing feature events. These two coupons avoid that policy only
through exact local topology; no admission, force, or acceptance rule is
weakened.
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
