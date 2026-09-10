# Accepted assembly wall archive v1

This is the new `source_assembly_wall` kind. It is a six-part component archive,
not a single-part schema or a full-vehicle claim. Production input comes only
from `SourceAssemblyWallCase::CaptureAccepted` and that case's same full accepted
stamp/diagnostics. Raw host formatter fixtures are not acceptance authority.

| File | Contract |
|---|---|
| `manifest.json` | `robo_dyna.source_assembly_wall_artifacts.v1`; published last after every inventoried file is closed and its bytes/hash rechecked. |
| `configuration.json` | `robo_dyna.source_assembly_wall_configuration.v1`; complete immutable source, case and wall declarations. |
| `source-assembly-inventory.json` | Exact authenticated original bytes, including all source cards, memberships, material/section/curve tables and released frontier. |
| `accepted-frames.csv` | Existing `owner_id,accepted_epoch,accepted_time_s,json_mesh,obj_visualization` header. |
| `accepted-NNNNNN.mesh.json`, `.obj`, `.fields.json` | Existing full-precision Chrono mesh and OBJ; fields use `robo_dyna.source_assembly_wall_frame.v1`. Width six is a minimum, never epoch truncation. |
| `accepted-intervals.csv` and declared segments | Exact `WallIntervalHeader` from `SourceAssemblyWallSchema.h`, one row per committed interval. |
| `final-metrics.json` | Actual `accepted_epoch`, `accepted_time_s`, `saved_frames`, complete stamp/diagnostics and allocation counts. |
| `original-canonical-wall.manifest.json`, `placed-wall.mesh.json`, `placed-wall.obj`, `placed-wall-placement.json` | Existing placed-wall schema and original vertex/triangle source identity. There is no `canonical-wall.mesh.json` alias. |

The manifest has `kind`, `status="completed"`, `shell_model=true`,
`vehicle_model=false`, `contact=true`, `horizon_complete`, `stop_reason`,
`owner_id`, `run_id`, `topology_id`, `source_instance_id`,
`source_inventory_file`, `source_inventory_sha256`, `source_inventory_bytes`,
`configuration_file`, `accepted_epoch`, `accepted_time_s`, `requested_steps`,
`frame_every`, `saved_frames`, shared `artifacts` and `interval_ledger_segments`.
A stopped prefix has a nonempty reason and actual epoch below requested steps.
All archives begin at accepted epoch zero. No failure gets a completion marker.

Configuration keeps `fixed_dt_s`, `requested_horizon_s`, requested counts,
IDs/hash, budget/forecast values and root `vertex_binding`/`triangle_binding`
from `AppendSurfaceBinding`. Its `input` object contains `parts`, `materials`,
`sections`, `curves`, `native_nodes`, `part_native_ledger`,
`internal_rigid_groups` and `boundary`, each table with explicit column names.
All original source IDs, source ELFORMs and thickness-point counts survive.
Each group has source group/set IDs, complete ordered member table, structural
and total M, native/physical/added J, centers, raw/effective tensors, principal
axes/inertias and regularization channels. `boundary` retains released rigid,
spotweld, external-node/part IDs and one-sided tied scopes with source location,
selected master/slave part sets and `pairing_qualified=false`. The inventory file
retains every original attachment card and external endpoint.

`wall_setup` contains explicit setup IDs, velocity, gap interval, law/error/area
settings, original wall filenames, exact physical/query coverage boxes, complete
1030 nodal-area certificates and 915 contact-parent area/share mappings. It
separates `native_initial_kinetic_J` and `aggregate_initial_kinetic_J`, plus
`group_initial_kinetic` and `generated_primary_mass_kg`. Every certificate is
`[value,lower,upper,error]`; primary mass is already included in each total group
mass. Exact field names/columns are frozen in `SourceAssemblyWallSetupFields.cpp`
and `SourceAssemblyWallInputFields.cpp`.

Frame root has the schema/kind, IDs/hash, accepted epoch/time, complete `stamp`,
`nodal_fields`, `sections`, `diagnostics`, and `contact`. Stamp names mirror
`NodalStamp`: `time`, `fixed_dt`, `velocity_time`, reaction timing, rotation
flags, and `rigid_groups={source_instance_id,group_count,member_count}`.
`temporal_scheme="staggered_half_kick_start"`; velocity phase is `collocated`
only initially and `previous_midpoint` later. `nodal_fields` stores flat
`position_xyz_m`, `velocity_xyz_m_per_s`, `orientation_wxyz` and
`angular_velocity_xyz_rad_per_s`. Positions have physical deformation scale 1.

`sections` is the unchanged complete `SectionFieldDocument`: all 915
`source_parents`, section values, three thickness points and native reported
thickness. Root `stress_frame="native_corotational_shell_axes"` explicitly
labels the native XX/YY/XY/YZ/ZX components; the section document's world
coordinate-frame string describes geometry, not a stress rotation to world.

`diagnostics` copies the case's shell, contact-onset, plastic and motion values.
`shells` holds QEPH/T3 accepted diagnostics and native base/current kinetic
arrays. `motion.before` is null initially; `motion.after` always exists. Each
kinetic sample names its phase: `physical_initialization` or
`stored_midpoint_with_lagged_frame`, with position/velocity/frame times in
seconds, ordinary/grouped native member channels and aggregate group channels.
Applied and reaction kick-work arrays name translation/rotation/total columns.
Reaction work is recurrence consistency, not dissipation; effective residual is
bookkeeping, not a collocated or independent physical-energy certificate.

The optional force-stage extension is described in
[`FORCE_STAGE_ARCHIVE.md`](FORCE_STAGE_ARCHIVE.md). Enabled configuration declares
`observe_force_stage=true` and every frame includes `force_stage_kinetic`: null
initially, then the completed force-stage observation belonging to that accepted
interval. Disabled configuration and frames omit both keys. Existing v1 schema
names, interval CSV columns, native work channels and byte reservations remain
unchanged.

`contact=null` only initially, reflecting certified separation rather than a
fabricated candidate. Later it has
`phase="prepared_candidate_of_accepted_interval"`, exact interval/source IDs,
global certificate/work/resultant values, and complete node/parent tables.
`node_columns` maps global/source node and original wall triangle IDs to three
certificates, world force/point/reaction/moment, power and flags.
`parent_columns` maps weight/source-parent index, EID/PID/MID/SID,
feature/face/family/arity, four local force certificates and resultant/potential.
The exact table orders are in `SourceAssemblyWallContactFields.cpp`.

Forecasting uses exact `PlanCsvLedger` bytes and segment counts for the requested
steps. Hard limits are 1 GiB total, 32 MiB per file and 1000 frames, reduced by
caller limits when requested. Per-frame reservation is 8 MiB fields, 1 MiB mesh
and 256 KiB OBJ. Static files and manifest are separately reserved. Forecast
failure precedes directory creation and accepted output capture. Frames must
include initial, requested cadence and final endpoints; a noncadence prefix
frame is terminal. Skipped, duplicate or wrong-base records reject.

The inherited shared CSV planner also limits each logical ledger to eight
32 MiB segments (256 MiB). At the reserved 1014 bytes per interval, this bounds
the current schema to roughly 264000 intervals, or 3.94 ms at h=1/67108864 s,
even when sparse frames would fit a larger total archive. Longer runs require
separate accepted checkpoint bundles or a separately qualified shared ledger
and reader extension; the 1 GiB aggregate allowance does not remove this limit.

The owning production include is `SourceAssemblyWallArtifacts.cmake`, target
`robo_dyna_source_assembly_wall_artifacts`. Pure host formatting/order/file tests
use `SourceAssemblyWallValues.cmake`, target
`robo_dyna_source_assembly_wall_values`; no engine or CUDA state owner is linked.
Standalone host qualification uses this parent directory's CMake with
`ROBO_DYNA_SOURCE_ASSEMBLY_WALL_OUTPUT_CHECK=ON`, plus the explicit frozen
inventory/wall, `ROBO_DYNA_TL_ROOT` and `Chrono_DIR`. The separate
`wall_artifacts/` CMake source owns the four real-case writer tests and requires
the qualified CUDA engine. It must be run through the shared workstation guard.

Author evidence: eight host tests pass against the complete authenticated source
with explicitly synthetic field values. Direct C++ compilation plus execution
took 7.587 seconds under one affinity CPU and a 512 MiB virtual-address cap,
with 372036 KiB peak child RSS. XML is
`/tmp/source-assembly-wall-artifact-host-check.xml`. The complete synthetic frame
fields occupy 4271584 bytes; actual immutable configuration occupies 1418862
bytes. Real-case tests (64-interval contact bundle, accepted prefix, late file
failure and preflight failure) passed bounded syntax only; owning builds and
CUDA execution remain the root coordinator's separate qualification gate.
