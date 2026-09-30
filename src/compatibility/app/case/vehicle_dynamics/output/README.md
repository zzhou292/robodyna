# Accepted frames from the physical dynamics owner

`VehicleAcceptedFrames` is a narrow case adapter to `PhysicalAcceptedFrames`.
It uses private friend access to the dynamics-owned startup and exposes only the
const output producer. Neither the public owner nor its clock or force buffers
are made available. The output module remains independent of the dynamics
implementation. The optional library composes the existing dynamics and capture
libraries without changing either execution schedule.

Capture always reads the common accepted endpoint. It may run between serialized
Prepare/Discard/Commit calls; a pending candidate is not a visible frame. All
original position/point/activity mapping and phase validation remain in the
existing producer. The archive writer continues to enforce the complete run
reservation, and adds no fabricated interval or completion record.

The owning original-model test captures startup, an uncommitted trial, discard,
retry and epoch one. It checks unchanged accepted fields before commit, actual
uniform-flight positions after commit, all 956,346 native plastic values and
349,645 activity flags, exact binary round trip, and unchanged device arenas.
This diagnostic trajectory has no wall/joints and lasts one 1e-8 s step; it is
not the requested deformation video.

Configure this directory with the same explicit original source inputs,
Chrono_DIR, TL root and CUDA architecture 120 as `case/vehicle_runtime`.
Target `robo_dyna_vehicle_dynamics_capture_check`; CTest
`vehicle_dynamics_accepted_capture`. Use the full-step 6 GiB GPU-growth guard.
The producer forecasts its additional capture workspace; caller-owned test
snapshots are separate from retained production storage.
