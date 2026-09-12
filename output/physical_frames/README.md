# Complete physical accepted shell capture

`PhysicalAcceptedFrames` reads the one `VehiclePhysicalStartup` owner and its
existing six-participant publication. Its private `CaptureAccess` bridge exposes
only accepted readbacks. The producer adds no solver state, clock, interval
admission, loads or mutable owner access. Calls must serialize with owner use.

Prepare an immutable `Mapping` from the retained `VehicleShellExecution`. It
preserves original shell node/EID/PID order, explicit QEPH/T3/QBAT indices and
coincident layers. Physical positions scatter through the actual ShellNodeMap;
serialized binary64 coordinates are never reconstructed from elapsed time.
LAW44 histories retain their true 1/3/4 point order. LAW1 is nonplastic, and
RigidSkin has zero constitutive points. Both are represented as unavailable for
plastic strain, rather than arrays of manufactured zero values.

The complete original declaration has 359,785 shell nodes, 349,645 parents,
1,037,877 actual material points and 956,346 stored native plastic values. The
remaining 81,531 points belong to elastic LAW1; 5,102 skins have no points.
The physical owner has 372,435 nodes. Activity is actual accepted parent mechanics,
separate from point failure or contact eligibility.

Capture validates the exact retained execution backing, common owner stamp and
all family phases before and after readback. It stages geometry, active native
fields and packed activity, then selects both records together. A late source,
point, phase or output validation failure preserves the previously visible pair.
Only active typed union members are read; object padding is never serialized.

`Preflight` charges the retained mapping/context, both frame slots, complete
physical x/v readback, reusable Q/T typed rows, QBAT rows, flags and codec scratch.
The source/physical model was prepared earlier and remains shared; its existing
reservation is separate. Mapping construction has its own 512 MiB cap and reuses
the existing bounded source-mapping factory. The capture cap is 512 MiB. The
archive source/codec cap is separate (128 MiB default); optional source-bundle
reader scratch remains bounded by its own existing SourceLimits.

`Archive::Prepare` reuses `PreparedSourceBundle` and `PlanWithActivity`. Its plan
charges the complete static inventory, every planned interval, sampled frames,
activity and the off-cadence accepted-prefix reserve within 2 GiB total and
32 MiB per file. Saved cadence is chosen independently of the integration step.
`Archive::Write` checks all five frame/activity destinations before writing,
consumes only actual increasing accepted epochs from that plan and forbids
further writes following partial I/O failure. It emits no completion manifest or
interval record. A planned horizon does not assert that an interval was executed.

This first freeze qualifies the capture/record seam. It uses existing binary
record readers and source mapping for replay; a separate activity-aware
FullShellFrameGeometry follow-up will control visible triangles and preserve its
existing stable original PID palette. No crash-video or complete-run claim is
made by an initial snapshot.

## Owning gates

Configure this directory with `Chrono_DIR`, explicit `ROBO_DYNA_TL_ROOT`, and
`ROBO_DYNA_PHYSICAL_CAPTURE_RUNTIME=OFF` for the small host gate. Target
`robo_dyna_physical_capture_values_check`; CTest `physical_capture_values`.
Author qualification uses one CPU/512 MiB and passes five functions, including
mixed 0/1/3/4-point mapping, nonzero independent point values, rollback/retry,
whole-buffer/whole-archive caps, six-family phase checks and late activity-file
collision before any frame write.

The root native/CUDA lane enables `ROBO_DYNA_PHYSICAL_CAPTURE_RUNTIME=ON` and
`ROBO_DYNA_PHYSICAL_CAPTURE_ORIGINAL=ON`, CUDA architecture 120, and the same
explicit original source inputs as `case/vehicle_runtime`. Build
`robo_dyna_physical_capture_original_check`. Run `physical_capture_forecast`
first and inspect its resource receipt; then `physical_capture_initial`.
The latter authenticates all source roles, constructs the real initial owner,
captures and compares every shell coordinate and plastic value, writes/rereads
the source bundle plus exact binary frame/activity, checks a late destination
failure/retry, and verifies the owner remains at epoch/time zero. No author
native/CUDA/full-source job has run for this freeze.


V5 capture additionally authenticates the optional eighth structural-beam
participant. `CaptureAccess` obtains source identity and parent count from the
actual retained Model and batch; `Phase` verifies accepted phase, owner,
configuration, epoch/base/attempt and all consumed times. Initial beam cache
must have no assembled interval. Later cache must report accepted force assembly.
Before/after capture checks preserve that source/count and endpoint. The public
frame format remains the existing exact shell geometry/material/activity format;
no beam history is fabricated or serialized as shell material data.

The host phase fixture covers optional TYPE45/beam combinations, initial and
accepted intervals, stale or missing beam source, late phase/attempt corruption
and successful retry. Full V5 source/model and accepted GPU capture remain the
owning root integration gate.
