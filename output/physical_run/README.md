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

New wall writers emit `robo_dyna.vehicle_wall_setup.v2` inside the same seventh
hashed wall file. Its bounded `physical_composition` receipt is derived from the
actual retained physical model: named domain/solid/ledger profiles, five typed
solid counts, physical node and rigid-group counts, selected point cards, and
immutable initial total/point mass with exact binary64 witnesses. The total is
the authoritative ledger total, not a sum reconstructed from diagnostic
subtotals. Joint census is explicitly unavailable from wall setup; the separate
run summary owns that observation. This source receipt does not certify a
current owner, energy balance, connectivity or completed physical trajectory.

`Replay::wall_composition()` returns a read-only nullable value. Historical
setup v1 remains readable and returns null, while v2 requires the complete
typed receipt. The reader checks profile/count coherence and canonical domain
bounds before mesh allocation and publishes the optional value only after all
later wall checks pass. Existing seven-file identities, per-file caps and
whole-run reservations remain unchanged; the small fixed receipt is covered
by the existing 32-metadata-block replay reserve and wall metadata workspace.

The focused owning target `robo_dyna_wall_composition_check` / CTest
`physical_wall_composition` covers both profiles, exact mass bits, malformed
late fields, unknown/mixed profiles and historical absence. Root's existing
`physical_run_wall_original` also checks actual V1 model values and a later
wall-plane rejection with its caller receipt unchanged. Full V4 wall output
requires the separately qualified V4 domain/model factory; codec fixtures do
not establish the original V4 node or rigid census.


## Vehicle supports V5

`Profile.beam18` is a trailing, default-false participant flag. The canonical
participant list appends `beam18` after optional TYPE45. Actual accepted interval
capture authenticates its retained Model/source/count and complete common
publication phase; it does not add beam work or energy columns. All previously
unavailable energies remain unavailable.

The V5 wall composition uses embedded `robo_dyna.wall_physical_composition.v2`
inside the unchanged outer `robo_dyna.vehicle_wall_setup.v2` file. It records
`OriginalVehicleSupportsV5`, the V5 coefficient order, 17 solid parts with
4980 parents ordered as `{908,1991,350,386,1345}`, 154 point-mass records, and
142 structural beams from four parts. Beam source and model profiles are
explicitly `OriginalCircularFourPointLaw44V1` and `CircularFourPointLaw44V1`.
The live composition factory authenticates the same canonical source, ordered
beam EID/PID rows, physical domain and retained beam coefficient producer.
Total mass comes directly from the complete ledger, including its existing beam
contribution; the receipt does not add diagnostic subtotals again.

V1/V4 composition output remains embedded v1, with no structural beam fields.
Historical outer setup v1 remains readable without composition when the run's
beam flag is false. Wall archive preparation rejects omitted or invented beam
presence even for initial-only prefixes. Replay requires the beam flag to agree
with a V5 composition and rejects missing, downgraded or forged beam metadata.
No additional files, archive channels or unbounded buffers are introduced; the
existing fixed metadata and wall workspaces cover the small appended fields.

The owning `physical_run_records`, `physical_wall_composition` and
`physical_capture_values` gates cover old/new participant combinations, exact
mass bits, schema downgrade, source/count/phase corruption and retry. The actual
V5 loaded archive gate belongs to `case/vehicle_run` and must run after the V5
runtime/CaptureAccess integration; codec fixtures alone do not qualify a live
beam trajectory. Root should retain the V1/V4 live regression alongside it.

## Frictionless self-contact accepted observations

`Profile.self_contact` defaults to false. Existing profiles and configuration
files remain v1 with identical field order and bytes. Enabling self-contact
selects profile/configuration v2 and appends `self_contact` to the participant
list. Generic energies and total contact work remain unavailable: the new
columns describe only the named contact participant and its actual phases.

`CaptureAcceptedInterval` requires self-contact presence to match the live
setup, runtime forecast and accepted observation. It checks physical source,
owner/configuration/qualification, active-use identity, attempt and accepted-base
force time/velocity phase, then requires complete candidate policy accounting.
The enclosing committed owner stamp authenticates the candidate's publication.
The pointer-free `SelfContactValues` stores 23 uint64 counts/identities and
17 binary64 force/potential/phase values. Contact forces and potential are from
the accepted **base**, while geometry policy and activity describe its validated
candidate. No endpoint force estimate, work integral or restart state is invented.

The existing interval integer/real files own every new column. There is no
untracked diagnostic sidecar. Self-contact rows occupy 392 bytes with a structural
limit, 384 without. `ExtraIntervalBytes` reserves 80/72 bytes beyond the existing
312-byte conservative interval reservation, so the expanded row forecast is
exact. Writer and reader share these chunk sizes and field lists. Preflight and
preparation normalize an omitted extra reservation once; any conflicting
explicit reservation rejects. Persisted v2 configuration requires the exact
reservation and rejects schema/profile downgrade or hidden columns. Whole-run
file-count forecasts retain the shared planner's conservative extra-file reserve.

Replay verifies finite force values, overflow-safe event/policy/activity
partitions, invariant source identity and exact previous-accepted velocity time.
The renderer continues to use the same geometry/native-plasticity/activity
frames; these additional interval columns do not alter displayed geometry.

`PhysicalRunSelfContact.*` host tests cover typed multi-chunk roundtrips including
integers above 2^53 and signed zero, profile/version/reservation corruption,
exact-cap versus one-byte-short rejection, invalid or overflowing partitions,
stale phase/source rejection, initial-only and sampled prefix validation, and
legacy profile bytes. These codecs do not substitute for a live committed
wall+self controller run.

## Native fixed-main contact observations

The additive `physical_observation_profile.v3` identifies the QEPH/T3/native TYPE25 participant set. It reuses the existing integer/real interval codec and exact accepted time-grid rules, with source identity/counts and accepted publication/reference generations plus the authentic force-base phase. It does not encode the earlier V5 residual-policy counters. Its fresh-owner publication generation must equal accepted epoch; references are positive, bounded by publication generation and increase at most once per interval. The initial frame is separate and has no completed interval. All unavailable force/work/energy fields stay explicitly unavailable. Native contact and legacy self-contact/joint/beam profiles cannot be combined by this profile.
