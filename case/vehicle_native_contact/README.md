# Full vehicle native contact case

This module composes the existing full-family physical source, genuine mixed
vehicle interface and the declared finite mesh wall. `VehiclePhysicalDynamics`
remains the sole owner, clock and step loop. Its native group hooks are owned by
the dynamics module. This worktree does not modify those hooks.

## Source boundary

The typed case factory accepts `EnvelopeOwnerSource`, `MixedStarterSource`,
`FiniteWallContactSource` and `InitializerControlsSource`. It verifies the exact
combined physical domain, the original prefix/canonical authority, the same
retained PostGapm source, and the same declared wall. Owning `SharesStorage`
queries establish shared retention before any budget subtraction. Geometry or
digest equality alone never grants a lifetime discount.

The self interface keeps its original selected NSV and mixed P/G/partner maps.
The new wall interface uses every retained vehicle physical node plus the four
wall nodes. Both contact sources use the actual complete physical domain. No
source node is added to model a generated rigid primary, no selected parent is
removed, and no material failure policy is disabled.

The authenticated controls table supplies original interface IDs and native
storage ordinals. The factory filters its TYPE25 entries to the selected self
and declared wall roles while preserving their source order. It verifies both
entries exactly once and never derives order from a role enum, a guessed ID or
an observed native table. The complete table, including TYPE2, remains available
to the initializer.

## Packing and initialization

Only absent runtime DTO arrays are packed. Topology, source coordinates, raw
origins, resolved supports, coefficients and pre-BUC gap fields are borrowed
from the immutable source handles. Node constraints come from the authenticated
complete owner packing already retained by the finite-wall source. The wall's
Starter cache and fixed-ready cache have distinct uses; no all-active cache is
installed as initial history.

The general TL initializer consumes the real ordered TYPE2 roster, contributor
census and source-proved complete native-population multiplier tier alongside
these source fields. Exact unused auxiliary IDs remain explicitly unavailable.
Its move-only DeviceSeed owns initial history/ICONT and finalized gap corners.
Transaction initialization creates it on the actual owner stream, copies it to
both existing transaction slabs, drains, and retires the seed. The app does not
create a host history facade or initialize warm rows to zero.

The case retains every borrowed input through the drain. Runtime initialization
reauthenticates the actual common physical binding, all mechanical participants
and `source.startup()`. After both transactions initialize, the case adopts them
in the authenticated declaration order and calls the private native-group
installer once. A failure destroys the incomplete case and its private contact
transactions; no accepted interval has been published.

## Resource phases

The complete forecast reports source construction peaks separately from current
retention. It charges the owner graph and contact graph conservatively, except
for duplicate owning handles proved with `SharesStorage`. Source packing,
physical startup and the two contact initializations are explicit phases. Both
runtime transactions coexist; initialization scratch/DeviceSeed for one
interface may retire before the next is built. The forecast must include whichever
real seed/runtime overlap TL initialization uses, not assume they are disjoint.

Use public physical, transaction and initializer forecasts. Do not copy CUDA/CUB
private layout formulas. Product host allowance remains 20,000,000,000 bytes;
current complete-source guards are 18 GiB sampled RSS and 4 affinity CPUs. Device
payload bounds do not include driver/context memory. Runtime allocation counts
remain explicitly incomplete where TL reports only exact bytes.

## Qualification sequence

1. Source admission and lifetime/budget checks, including exact shared backing,
   foreign domains, wrong wall, missing/repeated interface roles and cap edges.
2. Complete-source forecast with both real interfaces and the actual owner graph.
3. Initializer source/native/GPU gates, then actual startup diagnostics without
   stepping. Every warm-row/removal count comes from the producer.
4. Actual common-owner contact prepare/discard/retry and a short accepted prefix,
   with both group observations tied to one epoch and force-base stamp.
5. Existing RunLoop and accepted archive path, then Chrono replay. The output
   module's distinct full-vehicle native-group profile owns serialized validation;
   the small coupon's old native-contact profile is not reused.

## Owning source/forecast target

`VehicleContactStartup::ForecastPreparation` uses the same private preflight as
`Prepare`, before source-sized field and PreparedSource allocations. Its result
contains source/packing and host-preparation bounds only. `Prepare` then derives
both genuine final removal sets and obtains GeneralPreflight plans; only that
result has complete runtime and sequential-seed peaks. An aggregate fit of false
remains reportable and rejects `Initialize` before the physical owner is created.
`CensusInitialStates` is independent of that runtime fit, checks its own complete
bounds, and retires each genuine GPU seed before the next interface. Its host
bound conservatively includes source preparation. Census seeds are never reused
as runtime history.

The owning project is `actual/CMakeLists.txt`. Build only
`native_vehicle_contact_actual` and `native_vehicle_case_fields`. CTest runs only
the three small field groups. Invoke each full-source group separately with
`ROBO_NATIVE_VEHICLE_CASE_OUTPUT` naming a new directory:

- `NativeVehicleCaseActual.SourceAndHostPreparationForecastBeforeFieldAllocation`
- `NativeVehicleCaseActual.FinalRemovalPlansExposeCompleteRuntimeAndSequentialSeedPeaksWithoutOwner`
- `NativeVehicleCaseActual.SequentialGenuineInitialStatesReportWarmSignsAndTiedResetWithoutOwner`

They reuse the same four-handle actual source fixture, preserve lower failure
reports, and separately time source construction, host case preparation and GPU
source census. None creates a physical owner or advances a clock. The General
runtime successor must finish its owning gate before this source can qualify.
The subsequent accepted-run adapter will reuse the existing RunLoop and output
native-group profile; a source/forecast pass alone does not claim that run.

## Accepted native run

`PreparedRun` is a thin adapter over the existing `vehicle_run::detail::RunLoop`.
It retains the prepared case and actual environment mapping, preflights complete
capture/archive coexistence before owner creation, and selects the output
`native_group` profile. Its default 31 samples use the existing 2GiB archive cap;
the owning preview target explicitly selects `FullRunByteCap` and reports the
concrete forecast before a launch. It never silently shortens the requested
horizon or changes a contact/material policy.
The native output horizon is bounded at 100ms by `run/Horizon.h`; the existing
one-million-interval and authenticated contact-lifetime checks still run during
source preparation. Longer previews must separately fit the existing archive,
replay and process budgets. Increasing duration does not create physical restart.

The optional initial retry probe is outside timed runtime. It checks unchanged
accepted owner/group publication and exact frame/activity bits while a trial is
pending and after discard, then compares the two genuine attempt diagnostics.
It does not copy full contact history. A source failure, physics rejection or
I/O failure retains the real accepted endpoint and closes a truthful prefix
where the existing writer permits it. The owning preview test still fails if
the requested horizon was not completed.

The full actual target adds separate `CompleteOutputForecastBeforeTheOwner`,
`TwoNativeInterfacesRetryAndPublishTwoIntervalsWithClosedArchive`,
`ExplicitPreviewOutputForecastBeforeTheOwner`, and
`ExplicitPreviewHorizonUsesTheExistingRunLoopAndAcceptedArchive` groups. The
preview groups require explicit `ROBO_NATIVE_VEHICLE_DURATION_S` and
`ROBO_NATIVE_VEHICLE_SAMPLES`. They use the reviewed 5GiB steady payload cap,
unchanged 6GiB sequential initialization cap, and the 18GiB qualification guard.
Every invocation needs a fresh `ROBO_NATIVE_VEHICLE_CASE_OUTPUT`; actual accepted
archives live in its `accepted/archive` child with a separate viewer-input and
summary. Source construction, host case preparation, output preparation, session
startup, optional retry and timed runtime are reported separately. Detailed
per-stage counters remain available through `RunResult.loop.progress.mechanics_timing`
when the source config explicitly enables profiling; benchmark runs leave that
profiling disabled. Complete-domain uniform-motion departure includes the fixed
wall and is not labeled vehicle deformation. Saved native plastic fields and
accepted interface counters remain distinct observations.
