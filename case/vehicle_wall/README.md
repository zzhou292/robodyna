# Complete vehicle wall setup and initial contact attachment

`VehicleWallSetup` retains the exact immutable shell execution and physical CIN
attachment handles. It prepares all 349,645 QEPH/T3/QBAT parents on the original
359,785 surface nodes mapped into the 372,435-node physical domain. The existing
geometry adapter supplies native reference contact area; no new mass or inertia
is produced. The sole shared geometry change exposes its existing startup sum
as source, retained geometry and temporary fields. Legacy admission is unchanged.

The case declaration is 35 mph (`35 * .44704` m/s), a requested 20 ms duration,
20 mm leading gap, 10 mm transverse motion margin and 1 micrometre exposed-edge
clearance. The explicit numerical inputs default to stiffness/area `1e7`, maximum
penetration `.1 m`, and parent force/energy error `1e-6`. These values do not grant
loaded-step stability or predict the vehicle trajectory. The envelope extends
source X by the outward nominal speed-duration product and a margin; Y/Z receive
the declared margin. TL contact receives the separately projected box with both
X endpoints equal to the represented wall plane.

The original authenticated wall is always prepared through `PlacedCanonicalWall`:
62 vertices, 100 triangles, 46 source quads, original X `.05 m`, Y approximately
`[-1.0541, 1.0541]`, Z `[.066825, 1.759]`. The exact represented operation remains
`.05 + ((surface_max_x + gap) - .05)`. Complete original coverage is retained and
reported even when it fails. No source node, shell or layer is omitted to fit it.

The explicitly selected default `EnvelopeRectangleV1` is a new finite mesh with
four vertices and two consistently -X triangles. Its Y/Z extents add another
10 mm outward margin around the declared projected motion envelope. Shared
`PlanarWallGeometry` and box coverage validate the entire mesh. High-bit feature
IDs identify generated features; they are never original source IDs. Selecting
`PlacedOriginal` preserves the original mesh and its possibly unavailable
coverage. Uncovered setup cannot initialize contact.

`WriteSetupArtifacts` writes the original provenance/placed mesh and the actual
selected Chrono mesh/OBJ into an empty caller-owned directory using the existing
exact mesh archive/roundtrip writer. The separate setup receipt records source,
canonical, resolution and selected mesh hashes, original coverage, settings,
world envelope and feature identities. It is not a completed-run artifact.

`VehicleWallStartup::Preview` composes the existing dynamics preflight with the
mapped contact value forecast before owner/device allocation. Its empty prospective
publisher is only the existing forecast packet's non-owning placeholder; no
runtime proof/participant readiness is claimed. `Preflight` and `Prepare` then
require the actual dynamics owner, no pending attempt and the **same immutable
execution/roster storage**, using getter-reference identity rather than byte
coincidence. Contact `Initialize` performs its qualified full physical/publisher,
rigid/CIN and actual initial activity authentication. It allocates scratch only;
accepted epoch, all material histories and the common clock are unchanged.

This object does not modify free-flight `PrepareStep` or expose mutable owner
access. Ordered accepted contact force/STI assembly, combined current-response
screening, prepared work/activity capture and common discard/commit are the next
explicit wall dynamics integration. The initial test uses a `3e-7 s` constructor
candidate; loaded joints/contact may require a smaller admitted interval.

## Budget and lifetime

Setup defaults to an inclusive 8 GiB host cap, the existing explicit Vehicle
geometry profile and a 1 MiB source-wall manifest cap. It charges retained source
once, existing geometry payload and existing geometry scratch, actual manifest
copies, outer/control storage and a conservative 1 MiB bound for the tiny
original/generated wall containers and metadata. The source bytes are the same
named runtime `SourceBytes` bound, not the sum of older whole-construction caps.

The complete wall runtime defaults to 20,000,000,000 host bytes and 8 GiB device
payload, with contact's existing per-module 2 GiB host/device and 8 GiB inclusive
source limits unchanged. It charges the owner/source retained bound + app workspace
+ setup local retained bytes + contact local retained bytes + new control storage,
then the maximum of the independent geometry/owner/contact scratch phases. Exact
source identity is required before discounting shared backing. Device payload is
existing owner/participants plus contact arenas. The expected contact arena sum
is **1,087,555,232 bytes** at complete original counts; the owning source preview
must record the actual combined sum before the contact allocation gate.

These are payload/reservation bounds, excluding allocator/driver overhead. Root's
serialized GPU/RSS growth guards remain separate. Source handles outlive every
consumer. The existing dynamics owner must outlive the scratch adapter; calls are
serialized. Failure of a value-returning factory leaves an earlier caller handle
unchanged. No failed setup/readback grants a second owner or clock.

## Qualification

Author: 8 host functions pass (4 new wall values and 4 mapped geometry/forecast
checks), including wide/edge-on envelopes, exact coordinate operation, generated
identity/winding, invalid input/late overflow preservation and exact host/device
caps. Nine complete production/test C++ syntax units pass. Reports are
`crash-work/reports/vehicle-wall-author-final-1.{json,log,xml}` (10.027 s,
426,516,480 B sampled peak). Earlier syntax diagnostics were fixed and retained
in `vehicle-wall-author-syntax-{1,2}`. No original-source, Fortran or GPU run was
performed in the author lane.

Owning CMake source: `case/vehicle_wall`. Small target
`robo_dyna_vehicle_wall_values_check`, CTest `vehicle_wall_values`.
Set `ROBO_DYNA_VEHICLE_WALL_ORIGINAL=ON` for target
`robo_dyna_vehicle_wall_original_check` and separate CTests `vehicle_wall_setup`
and `vehicle_wall_startup` (one original function each). The setup gate reuses
the complete original fixture and therefore includes the already qualified
source search GPU dependency, even though it does not allocate a dynamics owner.
Run it first and inspect its complete host/device preview and original coverage.

Use the same `ROBO_DYNA_VEHICLE_{CANONICAL,SCOPE,DECLARATIONS,GLASS_RESOLUTION,
GLASS_SHA256,TYPE13_DECLARATION}` arguments as the original runtime gate, plus
`ROBO_DYNA_VEHICLE_WALL_MANIFEST=.../crash-work/assets/yaris-wall/manifest.json`.
`ROBO_VEHICLE_WALL_ARTIFACTS` optionally names an existing empty directory for the
setup test's create-only mesh/receipt export. The startup gate checks actual
initial owner identity, exact contact payload, short-budget rejection/retry and
unchanged accepted stamp. Neither test advances a loaded interval.
