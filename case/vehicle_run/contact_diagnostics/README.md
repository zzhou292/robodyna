# Optional contact substage diagnostics

Enable explicitly with `--self-contact-diagnostics` on `wall-self-contact-v1`.
The acceptance harness accepts `ROBO_SELF_CONTACT_DIAGNOSTICS=1`; absent or `0`
keeps its previous output. This is for the next necessary run, not a separate
vehicle startup just for profiling.

The bool travels through run config, contact composition and runtime startup to
TL's optional recorder. Existing app `StageTimer` is unchanged. Only the
runtime/observation adapter includes the new native diagnostic header; app value
DTOs use the existing `StageCounter` and standard types so host-only controller
builds retain their dependency boundary.

Native and observed contact stages forward the same read-only snapshot accessor.
A successful native seal returns before its copied diagnostics enter the step
observation. After common commit, the accepted accumulator checks owner, base
epoch, attempt, authentication and completed success flags. A mismatch makes
only the diagnostics unavailable; it never rejects otherwise valid mechanics.
Session retains its existing staged update, archive append, then summary publication.
The final run result separately copies TL's retained last attempt after rollback,
so a failed candidate is not mislabeled as the last committed interval.

Summary/progress fields are omitted when disabled. Optional summary objects add
no interval or physical archive fields, physical schema, solver clock, receipt
or publication authority. Nine fixed stage counters and bounded scalar discovery
counts fit the existing controller/summary allowances. No CUDA synchronization,
per-pair logging, dynamically growing sample list or physical branch is added.

Times are coordinator host elapsed scopes, including waits already present;
they are not GPU kernel times and must not be added to their inclusive parent
stage timings. Missing/backward clock samples and saturation are explicit.
Unavailable stage durations are omitted rather than represented as zero.
Counts describe executed calls or a traversed prefix, not a complete vehicle
census. `native_submitted_pairs` is the actual invoked native slice input count,
including a failing invocation; it differs from policy `exact_crossing_pairs`.
`native_work` is the admitted native report work, excluding later policy coverage.

Qualification reuses `vehicle_run_values`, `vehicle_run_reports`, and
`vehicle_run_observed_values`: explicit admission, disabled field omission,
phase matching, clock failure, bounded JSON, failure/published separation,
failed-append retention and decorator forwarding. Native instrumentation also
requires its owning TL tests; none of these host observations proves performance.
