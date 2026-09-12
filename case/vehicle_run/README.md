# Original physical run controller

`PreparedRun` orchestrates the existing `LoadedWall`/`VehiclePhysicalDynamics`,
accepted capture, and physical run archive. It creates one physical owner and
uses its actual fixed step, acceptance stamp, constitutive histories and clock.
The input is an immutable actual wall setup and its complete selected joint model.
The named loaded profile requires `EnvelopeRectangleV1` and at least 0.25 m
transverse margin, with the exact 35 mph source profile. The usual gap is 0.02 m;
the two-interval original qualification deliberately uses 1 µm to reach contact.

`Config` chooses a 0.5 ms preview or 5, 20, or 50 ms, a positive fixed step (default 3e-7 s) and 101 sampled
frames by default. The shared archive predicate selects the first mathematical
fixed-step endpoint at or beyond the requested duration. The last interval has
the ordinary fixed step; actual saved times come from the owner. The summary
keeps requested duration and actual completed time separate.

Preparation authenticates source/configuration identities, retains the exact
setup/joints/mapping, and forecasts before owner allocation. It checks the
shared wall backing once within the loaded-owner reservation. Mapping startup
has a 512 MiB cap; capture/writer retain their existing bounded utilities. The
inclusive host reservation takes the larger mapping phase and simultaneous
loaded-owner/capture/archive phase, plus 4 MiB controller/summary workspace.
The separate summary reserves 1 MiB and viewer-input receipt 64 KiB in addition to
the strict archive. Normal limits remain 20 GB host and 2 GiB archive. An explicit
`ConditionalExpandedFull` request selects up to 60 GB/6 GiB only when the complete
forecast exceeds a normal cap. It does not weaken any participant's native cap.
Device reservation is the actual loaded runtime forecast; no second device
state or copied source geometry is created.

`Execute` requires a real empty destination directory. The strict archive lives
in `archive/`; `run-summary.json` and `viewer-input.json` live beside it. The
sequence is initial accepted capture/sample, then Prepare/Commit, authenticated
accepted-row append, and captures only at the declared archive cadence.
A cooperative stop, diagnostic interval limit, elapsed-time boundary, or
physics rejection captures any unsampled final accepted endpoint and closes an
explicit prefix. An append/sample/final-write failure stays incomplete; it does
not manufacture a successful prefix. An accepted readback failure is distinct.
The viewer receipt is create-only after a valid Finish/FinishPrefix and binds
its manifest, original source authority and mapping digest. WithWall Append
also requires the exact immutable setup from the actual loaded dynamics.

A StepSizeError preserves its actual reported limit and physical node. The
controller does not change the owner's fixed step or reuse visualization as
restart state. Recovery is a new source-based run into another empty directory
with an explicitly selected smaller fixed step; the old prefix remains intact.
Contact rejection preserves the actual status/node/parent without inventing a
step-size estimate. Caller progress/stop functions run at accepted boundaries;
an elapsed-time limit starts after startup and cannot interrupt a CUDA stage.

Progress includes elapsed wall time, accepted throughput, contact endpoint
peaks and the existing StageTimer total/last-attempt snapshots. The summary also
reports startup wall time, successful host-call duration totals and final stage
counters. Inclusive PrepareStep must not be added to its child stage durations;
use last-attempt or snapshot deltas to separate TT0/retry from later steps.
Contact values are reported endpoint peaks, signed reported drift-work sum,
separate same-mask/removal potential and actual/proposed activity counts.
They are not continuous-time maxima or a total-energy balance. The current
versioned interval schema still marks contact/energy columns unavailable.

The additive `accepted_mechanics` object in `run-summary.json` reports existing
committed scalar diagnostics. It preserves the outer summary v1 fields and
uses its own `robo_dyna.accepted_mechanics_summary.v1` schema. Initial-only
prefixes say `available=false`; archives and older summaries need no new fields.
It records all five solid families in their existing order and the optional
structural beam's membrane/shear and flexural/torsional channels. Each work
channel has its last signed increment, accepted-increment sum and absolute
increment peak. These sums exclude TIME0 energy. Solid LAW36/LAW44 plastic work
is one combined existing diagnostic, not a new per-family split. Plastic and
hourglass work are included components of native work and must not be added to
it again. Actual RHS kick/drift work remains separately labeled.

The first positive reported plastic-work increment includes the actual accepted
epoch/time. It does not identify a point plastic-strain maximum, damage or a
permanent shape change. Native element timestep minima are observations, not
the post-CIN/contact admission bound. Motion fields reuse the complete-domain
component maxima relative to original uniform translation; during impact these
are departures from that reference, not errors that should stay zero, fitted
rigid-motion residuals or strains. No new magnitude threshold stops mechanics.
No total/kinetic energy or energy-balance claim is introduced.

The summary consumes `last_accepted_step()` after the existing actual interval
authentication, without GPU readback or a new clock. Missing/stale participants,
changed source/counts and nonfinite or overflowed values reject the update
transactionally. Only a successful archive append publishes the new contact
and mechanics summaries. Fixed scalar storage (at most 4 KiB per summary plus
bounded copies) fits the existing 4 MiB controller reservation and the existing
summary byte cap. Normal 20 GB host and 2 GiB archive defaults are unchanged.

Detailed solid/beam point-history export and a front-detail replay camera remain
separate work. Current frame plasticity covers shells only. Diagnostic stop
limits do not change the archive sample schedule, which spans the requested
full horizon; the last accepted endpoint is always sampled before prefix export.

The private Operations seam is only for deterministic loop fault tests. It is
not a public alternate solver. Host gates cover ordering, cadence, stop/failure
prefixes, failed commit/output, overflow, source-phase summary checks and caps.
The optional root-owned original gate uses all 372435 physical nodes, 349645
shells and 38 joints, executes two loaded intervals, and reopens the archive and
viewer receipt. It does not claim a completed 5 ms trajectory.

Build the pure loop checks from this directory. Enable
`ROBO_DYNA_VEHICLE_RUN_REPORT_VALUES` for host contact/summary checks, with the
qualified `Chrono_DIR` and `ROBO_DYNA_TL_ROOT`. The live target is
`robo_dyna_vehicle_run`; enable `ROBO_DYNA_VEHICLE_RUN_LIVE`. Root may also enable
`ROBO_DYNA_VEHICLE_RUN_ORIGINAL` with the existing complete source fixture paths.
CTest entries are `vehicle_run_values`, `vehicle_run_reports`,
`vehicle_run_forecast` and `vehicle_run_loaded_prefix`. Set
`ROBO_VEHICLE_RUN_OUTPUT` to a pre-created empty directory to retain the short
qualification's archive for the viewer; otherwise the test uses a temporary.

## Standalone original-source launcher

The live build also provides target `robo_dyna_vehicle_run_cli`, executable
`robo_dyna_vehicle_run`. It has no GTest dependency. Run `--help` for all options.
`source/OriginalSources.cpp` retains the qualified pinned canonical manifest,
current scope report, main/auxiliary/original-wall member and declaration
identities. `OriginalYaris.cpp` composes the existing source, material,
reference, physical-domain/ledger, tied-classification, joint and wall factories.
There is one canonical backing and no card/geometry parser in this frontend.
The main source text is read once and passed to the canonical reader and typed
source consumers. Local setup temporaries are released before the live session;
the immutable setup and joint model retain their required backing. Each factory
keeps its existing source/reference/startup preflight and default caps. The
source-only search assessment still uses its qualified bounded GPU broadphase,
so even `--forecast-only` belongs in the guarded root execution lane.

All paths are explicit; extracted main/auxiliary/original-wall members can be
staged with the existing authenticated source utilities. The launcher does not
unpack an archive, download anything, mutate source files or create a second
solver. The destination must already exist and be empty. Example invocation
(after supplying the nine source path options shown by `--help`):

```
robo_dyna_vehicle_run <source path options> --run-id 1001 --output /path/to/run-1001 \
  --duration-ms 5 --fixed-dt-s 3e-7 --gap-m .02 --samples 101
```

Use `--forecast-only` first, then a separate empty destination with
`--diagnostic-intervals 2` for a short accepted prefix. The ordinary 20 mm gap
will be free flight at those first endpoints; explicitly use `--gap-m .000001`
only for the first-contact qualification. With no diagnostic limit the same
loop executes the complete planned horizon. A later 20/50 ms run requires a new
setup/owner/output directory and does not resume the prior visualization.

For a dense complete-assembly preview, select
`--physical-profile vehicle-supports-v5 --duration-ms 0.5 --fixed-dt-s 2e-7 --samples 101`
with the same authenticated source paths and a fresh run ID/empty directory.
Use its forecast first, then run without `--diagnostic-intervals` to request the
whole preview. The existing binary64 horizon rule selects 2,501 ordinary fixed
intervals and 101 sampled accepted endpoints, including epoch zero and the final
epoch. The wall envelope uses that declared 0.5 ms duration. Actual saved times
come from the owner; no frame interpolation, shortened step, velocity change or
mass scaling is introduced. Completion still requires the final archive receipt;
a stop/rejection remains an explicitly incomplete accepted prefix. A subsequent
5 ms run starts from the original state in another directory. A 2,500-interval
diagnostic stop on a declared 5 ms horizon keeps the 5 ms cadence and has only
11 scheduled frames, so it is not the dense-preview configuration.

`--wall-stiffness-n-m3` and `--penetration-limit-m` explicitly override the
existing wall-case declarations for contact sensitivity runs. Stiffness has
units N/m³ because contact multiplies it by the retained reference area and
penetration. Both options require finite positive values and are written through
the existing wall setup artifact. If absent, the owning case keeps its defaults.
These controls do not bypass the physical/contact timestep screens or change
the fixed integration step. A penalty choice still needs a demonstrated
penetration and deformation response before calling the impact sufficiently
close to a rigid wall.

`--maximum-elapsed-s` stops at a completed accepted boundary after startup.
`--stop-file PATH` supplies a simple cooperative stop: create that file outside
the archive and the next accepted boundary exports a prefix. It does not
interrupt an in-flight CUDA call. `--conditional-full-limits` merely permits
selection of the larger already authorized ceilings if the measured forecast
requires them. Exit 0 means the whole planned horizon and its artifacts finished;
exit 2 means a valid accepted prefix; exit 3 means capture/archive/summary/receipt
failure; exit 1 means source/configuration/startup orchestration failed.

## Explicit expanded physical profile

`--physical-profile extended-solids-v4` selects the original 4,900-solid source
profile, rebuilds its canonical-ordered physical domain, and restores the two
newly supported spherical joints. The default `retained-shell-v1` remains the
qualified 2,412-solid / 38-joint case. The expanded profile has 40 joints; its
four rod-joint boundaries, missing structural beams and airbag solid cells remain
explicit. It does not imply a completed long crash trajectory.

The run controller checks the actual sealed domain and joint policies before
forecasting or allocating an owner. Profile selection does not expand resource
allowances. The existing normal 20 GB host and 2 GiB archive caps remain in force;
conditional full-run allowance is selected independently and used only when
required by the complete forecast. Run summaries identify the selected profile
and actual prepared joint count. The existing typed source/model/attachment
factories, CUDA owner and Chrono replay remain the implementation path.

The V4 source, full owner, two loaded intervals and authenticated replay passed
the root qualification at app `3ea02b3`. This is a short accepted prefix, not a
completed 5 ms trajectory.

## Complete vehicle supports profile

`--physical-profile vehicle-supports-v5` selects the sealed V5 source/domain:
4,980 solids (908/1,991/350/386/1,345 in the five existing families), 142
structural beam18 parents, 154 point-mass records and all 44 retained joints.
The beam source keeps its original two force endpoints; N3 remains orientation
evidence. `PhysicalSelection` pairs the original solid, physical-domain and
joint policies. The source factory reuses the immutable original beam source
and named physical-scope factory; the controller rejects a profile whose actual
sealed model, joint policy or beam availability differs.

Runtime adds one distinct beam18 participant to the same nodal owner and common
publisher. Its native stiffness/forces/couples enter the accepted assembly before
CIN transfer and the existing structural screen. Candidate histories commit or
discard with all other participants. The archive profile reflects actual beam
availability and retains the authenticated V5 wall composition. No new clock,
source parser, beam spring surrogate or separate publication is introduced.

Root's source/model/CIN gate measured 376,930 nodes, 779 rigid groups with 12,961
members, and 704.35196065175535 kg including 1.5537032763072669 kg from beam18.
Production derives counts from those retained sources. This runtime increment's
full-source checks remain root-owned: `vehicle_run_supports_forecast`, then
`vehicle_run_supports_initial`, then `vehicle_run_supports_loaded_prefix`. The
last gate requests 5 ms but deliberately ends at two accepted 2e-7 s intervals
with a 1 µm gap, keeping the actual post-CIN timestep screen enabled. A rejection
records the measured limit; this authoring does not certify the selected step.
Normal host/archive/device caps and default/V4 selection remain unchanged.
