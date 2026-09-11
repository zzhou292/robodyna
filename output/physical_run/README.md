# Physical accepted run archive

This output layer records accepted physical-owner endpoints. It owns no solver,
clock, contact or joint mechanics. `RunArchive::Append` accepts only an
`AcceptedInterval` returned by `CaptureAcceptedInterval` from the actual dynamics
and its authenticated capture producer. Capture cadence is independent of the
fixed integration step; the row factory does not download a frame every step.

`MakeRequest` and `Preflight` reserve the complete source bundle, all interval
chunks, planned frames/activity, one possible final off-cadence prefix frame,
and final metadata. The default whole-run cap remains **2 GiB**. An explicit
`records::FullRunByteCap` permits **6 GiB** only when a full-run forecast needs
the conditional larger allowance. Per-file 32 MiB and 64 interval-chunk limits
remain unchanged. Byte totals use checked 64-bit arithmetic; the host test
exercises an actual forecast above 4 GiB, exact-cap admission and one-byte-short
rejection. No six-GiB array or archive is allocated by that value test.

The physical interval profile stores owner/epoch/base/attempt as uint64 and the
four native phase times as binary64, with one optional actual structural-limit
scalar. Kinetic energy, total energy, internal work, contact force/penetration/
work, and joint work are explicitly unavailable and have no numeric columns.
Versioned profile metadata names the optional seventh TYPE45 role. Capture
checks its actual retained model/count, batch and accepted publisher stamp;
the live row factory requires profile presence to equal the actual common
publisher and validates source, count, phase and initialized automatic stiffness.
Initial capture requires automatic stiffness to remain uninitialized.

Call `Sample` for the initial frame and each requested accepted sample, `Append`
after common acceptance, and `Finish` only after the complete planned horizon.
`FinishPrefix(reason)` requires the exact final accepted frame and records the
shorter endpoint, including an initial-only prefix. Input validation can retry.
An actual output I/O failure is sticky: partial create-only files remain evidence
and cannot be published as a completed archive or valid failed prefix. The outer
manifest is written last. No crash-duration or video claim follows from a prefix.

`Replay::Open` requires the expected outer manifest hash, original source inputs
and mapping digest. It authenticates every source file, interval, frame and
activity record, including the complete referenced-file inventory, before
publishing its immutable handle. `ReadSample` rechecks hashes. Its existing
mapping/context/frame/activity types feed `FullShellFrameGeometry` with original
PID colors, exact binary64 geometry, true 0/1/3/4-point layouts and explicit
unavailable plastic fields. A zero-length typed plastic array is valid for a
fully nonplastic sample. There is no interpolation, restart or energy inference.

The default reader cap is 512 MiB, writer cap 256 MiB. These are archive-owned
source/record staging budgets; the live GPU owner/capture and any caller-held
samples remain separate resources. The reader streams chunks instead of
allocating the complete interval history. Its complete original-source peak
and initial/one-interval gates are authored and require root execution.

The optional named wall adapter uses `MakeWallRequest`,
`RunArchive::PreflightWithWall` and `PrepareWithWall` with the actual immutable
`VehicleWallSetup` and `physical_frames::Mapping`. It authenticates their exact
execution backing and requested duration before mutation. `WriteSetupArtifacts`
produces exactly seven files in `wall/`: original canonical manifest, placed
mesh/OBJ/placement, selected mesh/OBJ, and setup. All seven reserve 1 MiB each
under the selected whole-run cap. The original 62-node/100-triangle wall or the
selected four-node/two-triangle rectangle bounds serialization before writes.
The writer reserves 16 MiB for wall serialization; closing requires a complete
typed receipt in the outer manifest. Default no-wall calls remain strict.

Replay checks complete file hashes, pinned original wall provenance, vehicle
source associations, placement/selected mesh linkage and exact plane bits. Its
`wall()` receipt and `wall_mesh()` expose the authenticated immutable mesh.
Neither reconstructs geometry from settings. The reader reuses the geometry-only
Chrono reader extracted from `AcceptedReplayMesh`; legacy readers retain their
previous 4096/8192 count limits and coordinate/topology checks. Record and wall
temporary phases are sequential, so the budget takes their maximum and separately
charges 512 KiB for retained bounded wall geometry. Unknown companion files remain
rejected even with a rehashed outer inventory.

The existing mesh writer rejects a negative-zero coordinate if Chrono's JSON
integer encoding loses its sign. The regression retains that rejection; binary
shell frame arrays independently preserve signed zero exactly. No value is
silently canonicalized to make a mesh pass.

## Qualification

Owning pure target: `robo_dyna_physical_run_check`; CTest `physical_run_records`.
Fourteen host functions cover roundtrip, exact binary64/uint64, cadence/epochs,
prefix/completion, optional availability, chunk/host/whole-run caps, rehashed
phase corruption, unknown referenced inventory and an actual short write.

Configure `output/physical_run` with explicit `Chrono_DIR` and
`ROBO_DYNA_TL_ROOT`. The pure target needs no CUDA execution. To build root's
actual gates, also set `ROBO_DYNA_PHYSICAL_RUN_LIVE_FACTORY=ON` and
`ROBO_DYNA_PHYSICAL_RUN_ORIGINAL=ON`, with existing original fixture arguments
`ROBO_DYNA_VEHICLE_{CANONICAL,SCOPE,DECLARATIONS,GLASS_RESOLUTION,TYPE13_DECLARATION}`
and `ROBO_DYNA_VEHICLE_GLASS_SHA256`. The original CTest names are:

- `physical_run_InitialOnlyPrefixHasCompleteSourceAndNoAcceptedIntervalClaim`
- `physical_run_OneActualAcceptedIntervalFactoryDiscardAndFailedPrefixRoundTrip`
- `physical_run_OriginalJointInitialPrefixAuthenticatesSeventhSourceAndVirginPhase`

The advancing-prefix test performs one explicit diagnostic freeflight interval and a discarded
attempt. It makes no joint/contact/full-crash claim. Author checks are one CPU,
512 MiB host-only; all actual source/GPU checks remain root-owned.

Affected shared gates are `full_shell_visualization_records`,
`full_shell_interval_segments`, `full_shell_canonical_array_compatibility`, and
`physical_capture_values`. Existing frame archive compilation is factored into
`robo_dyna_physical_frame_archive`; the runtime capture API is unchanged.

For the actual wall gate, also enable `ROBO_DYNA_PHYSICAL_RUN_WALL=ON` and supply
`ROBO_DYNA_VEHICLE_WALL` (the pinned original wall manifest). The additional
target is `robo_dyna_physical_run_wall`, CTest `physical_run_wall_original`.
It checks initial-only complete-source archive/readback, exact wall coordinates,
late file corruption/retry and one-byte-short host admission before file mutation.
This gate is authored for root; it makes no loaded contact claim. The shared mesh
extraction also affects `robo_dyna_accepted_replay_check` / CTest `accepted_replay`
and retained wall replay fixtures.

Loaded accepted rows additionally retain a typed immutable `VehicleWallSetup`
backing identity. Only the actual dynamics factory can seal that identity after
checking both same-attempt wall force/candidate observations. `Append` requires
the exact setup retained by `PrepareWithWall`; a reused numeric wall ID, a
separately prepared equal setup, a changed placement, or a free-flight row cannot
stand in for that authority. No wall serialization or digest is repeated per
step. Initial-only wall prefixes remain valid and claim zero contact intervals.
The handle retains existing setup/source storage; the forecast exposes its
`shared_wall_setup_upper_bound` separately for composition to charge once.
The writer's incremental workspace bound includes the small handle in its
existing bounded object/metadata allowance.

The follow-on owning `physical_run_wall_phase` checks ten corrupted observation
phases with retry. `physical_run_wall_original` additionally contains actual
loaded seven-participant acceptance with foreign-placement rejection (even with
a reused wall binding ID), and actual free-flight rejection. These source/CUDA
functions are authored for root qualification; author checks alone establish no
loaded archive result.
