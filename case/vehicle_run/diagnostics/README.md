# Optional self-contact failure diagnostics

`--self-contact-failure-output ABSENT_DIR` records bounded evidence from an
actual rejected wall+self-contact attempt. It works with the ordinary CLI
horizon and controls; it does not introduce another run loop or solver.

`FailureRun.h` exposes only destinations, forecasts, ordinary run results and
diagnostic status. The private implementation reuses `observed::RunAccess`, its
native contribution bridge, and the existing candidate-failure codec. The CLI
never receives a callback, owner token, transaction or initialization hook.
The GTest-free support libraries are separated from test registration; their
qualified implementation sources remain in place to avoid unrelated movement.

Before owner construction, the CLI validates a new companion path outside the
run and source inputs and admits an additional 8 MiB host capture allowance,
1 KiB wrapper reserve and 2 MiB possible fixture within the selected caps.
`--forecast-only` includes those costs when the option is present and therefore
requires an explicit `--output` path too. No device reservation is added.

The native observer freezes a supported authenticated pair synchronously before
rollback. The normal controller then closes its accepted prefix. Publication
uses the existing bounded writer/readback and compares the captured typed
failure and accepted epoch with the unchanged controller result. Diagnostics
live outside the immutable run archive. Existing/partial companions are never
overwritten. Success leaves the requested companion directory absent.

Reported statuses are `no_rejection`, `captured`,
`rejection_without_authenticated_pair`, `capture_incomplete`, and `export_failed`.
Missing-pair and incomplete states do not claim a replayable fixture. The current
native hook covers prepared intersection policy, mapped native crossing failure,
and final candidate policy. Earlier discovery/activity/resource failures, accepted
assembly errors and external termination may have no authenticated pair.
Capture remains terminal-only: freezing accepted policy can invalidate current
regularity authority and must never become a progress callback for a continuing
attempt. The original typed rejection and accepted prefix remain authoritative.

Normal CLI exit conventions are unchanged. Requested capture/export failure
returns artifact-error code 3 after printing the original physical result; a
valid rejected prefix with either a captured or unavailable pair remains code 2.
Neither successful export nor code 2 indicates completed crash acceptance.

Host tests `vehicle_run_failure_diagnostics` cover path protection, composed
budgets, later-epoch report matching, missing evidence, and export/error
propagation. `vehicle_run_values` covers option admission; existing observer and
frozen-fixture tests retain ownership and codec coverage. Host publication tests
use inert evidence and exporters and do not claim native physical acceptance.
A native multi-step rejection remains a separate GPU qualification.
