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
Versioned profile metadata reserves a named seventh TYPE45 role; this first
core freeze's live factory admits six participants. The follow-on consumes the
qualified seventh publisher API and checks the actual batch/accepted stamp.

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

The core schema is deliberately no-wall. A named `WallCaseArtifacts` follow-on
will retain the actual `VehicleWallSetup` receipt in `wall/`, with exactly the
seven files produced by `WriteSetupArtifacts`: original canonical manifest,
placed mesh/OBJ/placement, selected mesh/OBJ, and setup. It will add fixed typed
reservations and a manifest receipt, bind original source/execution identities,
and expose the authenticated selected mesh. Unknown companion files remain
rejected. No wall is reconstructed from untrusted settings.

## Qualification

Owning pure target: `robo_dyna_physical_run_check`; CTest `physical_run_records`.
Twelve host functions cover roundtrip, exact binary64/uint64, cadence/epochs,
prefix/completion, optional availability, chunk/host/whole-run caps, rehashed
phase corruption, unknown referenced inventory and an actual short write.

Configure `output/physical_run` with explicit `Chrono_DIR` and
`ROBO_DYNA_TL_ROOT`. The pure target needs no CUDA execution. To build root's
actual gates, also set `ROBO_DYNA_PHYSICAL_RUN_LIVE_FACTORY=ON` and
`ROBO_DYNA_PHYSICAL_RUN_ORIGINAL=ON`, with existing original fixture arguments
`ROBO_DYNA_VEHICLE_{CANONICAL,SCOPE,DECLARATIONS,GLASS_RESOLUTION,TYPE13_DECLARATION}`
and `ROBO_DYNA_VEHICLE_GLASS_SHA256`. The two original CTest names are:

- `physical_run_InitialOnlyPrefixHasCompleteSourceAndNoAcceptedIntervalClaim`
- `physical_run_OneActualAcceptedIntervalFactoryDiscardAndFailedPrefixRoundTrip`

The latter performs one explicit diagnostic freeflight interval and a discarded
attempt. It makes no joint/contact/full-crash claim. Author checks are one CPU,
512 MiB host-only; all actual source/GPU checks remain root-owned.

Affected shared gates are `full_shell_visualization_records`,
`full_shell_interval_segments`, `full_shell_canonical_array_compatibility`, and
`physical_capture_values`. Existing frame archive compilation is factored into
`robo_dyna_physical_frame_archive`; the runtime capture API is unchanged.
