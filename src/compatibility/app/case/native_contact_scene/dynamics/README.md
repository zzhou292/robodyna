# Production native shell-impact runner

This module composes existing TL owners/operators. It imports no native observation fixture, Fortran reference or GTest into the CLI. ContactSource supplies genuine numerical startup fields from the declared JSON mesh/material; NativeSceneDynamics owns exactly one FENodalState, QEPH/T3 participants, common publisher and native Transaction. An initial material-cache proof is assembled and discarded without any physical advance or contact history evaluation. The real scratch roster is bound before stepping.

PrepareStep uses three closed internal stages: material accepted assembly, native accepted contact assembly, then ordinary complete structural/contact screen and material/common candidate seal. All fallible work precedes CommitPhysical. Only successful common commit flips the app observation selector. Discard and destruction revoke an outstanding trial before releasing contact, publisher, batches and owner. The same stages have a private owning-qualification friend; it observes before/after contact RHS/STI and cannot manufacture a receipt. There are no product callbacks or extra per-step physics copies.

Preflight uses the existing owner, shell and publisher forecasts. The native transaction's public API has no separate preflight; its explicit complete host/device caps are reserved before owner allocation, and its own exact internal admission/allocations are checked against them afterward. Those caps are honest upper bounds, not exact usage predictions. Source/capture/archive module bounds conservatively retain their shared-source inclusions; allocator and CUDA driver/context memory are outside payload forecasts and remain guarded externally. Source construction precedes runtime and is bounded by each source module's own limits.

NativeAcceptedFrames retains two complete physical-domain readback slabs (X/V/quaternion/spin/reactions/raw M/J), native history/ICONT and actual phase metadata. It publishes them only with the same successful frame/activity selector after all reads and before/after source/publication checks. Raw arrays are copied only when Capture is requested; production unsampled steps use the small accepted metadata interval query. Initial reactions are unavailable by the actual stamp. Native history and supplemental physical arrays are typed observations, not physical restart state.

PreparedNativeSceneRun uses the unchanged vehicle_run::detail::RunLoop orchestration seam with its own native session. It constructs the shared exact-step horizon directly and never selects a Yaris profile. The loop guarantees initial/scheduled/terminal samples, actual accepted append, cooperative stop, and complete/prefix closure. Capture or archive failure is recorded separately from physical rejection; a successful physical endpoint does not imply successful archive closure. Only the live AcceptedInterval factory authorizes Append.

The CLI requires --source, --source-sha256, --source-output, --output and --run-id. Both output directories must be explicitly pre-created empty real directories. Source-output is a separate static staging artifact; it must not equal the physical run directory. Forecast-only writes its forecast there and creates no GPU owner. Use a fresh source-output for a subsequent actual invocation so frozen source/forecast artifacts are never overwritten. Defaults are1000steps,300ns,31states, with source-derived timestep safety and no mass scaling. Diagnostic stop limits do not change the declared horizon. The source step cap and complete STI screen still apply.

The summary schema is robo_dyna.native_shell_impact_run.v1. Its caption is GPU shell-impact coupon; it explicitly disclaims vehicle delivery and restart scope. The existing ViewerInput and physical archive codecs feed the existing Chrono reader. Actual recorded timestamps and scale1 coordinates remain authoritative; playback cadence is separate from physical time.

Owning gates include eight host CLI/forecast groups, three actual GPU initialization/rollback/destruction/archive groups, and the full1000interval generated-source versus independent native trajectory/RHS/STI/history/packet comparison. The latter preserves the original numerical tolerances and uses observations only for assertions. Source records, old archive regressions, no-GTest/no-Fortran CLI linkage, actual CLI closure and exact Chrono replay are separate required gates before publishing a video.

## Moving main surfaces

The runner retains a `ContactSelection`, a closed immutable choice of the two
already compiled source factories. A v1 fixed-wall declaration selects
`ContactSource`; an explicit v2 all-shell declaration selects
`MovingContactSource`. Its typed visitor selects the corresponding TL
`Transaction::Initialize` overload once at startup. Both choices use the same
physical owner, batch participants, prepare/commit/discard implementation,
controller and accepted archive capture. There is no per-step host dispatch.

The forecast and summary record the actual selected native profile. Existing
fixed-profile keys and values remain unchanged. Moving runs retain genuine
Starter normals, with subsequent cache updates owned by the native transaction.
Raw declaration hashes bind the selected source membership. They do not assert
whole-vehicle readiness or a matched CPU/GPU performance win.

The moving trajectory gate consumes only independent expected data through the
existing `ReferenceReader`; source factories and runtime never read that fixture.
Shared assertions preserve the fixed gate's original tolerances. The new full
1,000-interval gate also discards/retries an actual force-active attempt, checking
all accepted physical fields, native history and normal-cache bytes before it
allows the single common commit to advance.
