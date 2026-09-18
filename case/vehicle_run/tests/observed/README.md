# Observed controller qualification

This test-only composition lets an existing `PreparedRun` freeze a native
self-contact rejection during its ordinary execution. It does not introduce a
solver, owner, clock, alternate run loop, or global/environment solver setting.
The production CLI and `PreparedRun::Execute` use the unchanged null-hook path.

`RunAccess::Execute` first admits an additional 8 MiB capture allowance, 1 KiB
contribution reserve and 2 MiB possible export against the configured complete
host/archive caps. It constructs `CandidateFailureFixture` using the prepared
run's actual retained transaction limits. No additional device storage is
requested. The returned `ObservedResult` retains the normal run result, exact
qualification reservation, and fixture; the caller may inspect or export the
fixture after execution has closed its accepted archive prefix.

The private initialization hook runs immediately after the existing contact
factory creates dynamics, before frame capture or archive construction. It
allocates the decorator before moving the original concrete contribution, so a
failed installation cannot abandon its authority. The decorator forwards
assembly, scratch receipts, discard, setup, forecasts, and allocation reports.
Only candidate sealing dispatches to TL's existing bounded failure observer.
The bridge retains the shared accepted-observation check, completed-observation
publication and exact typed stage error. Normal dynamics therefore owns all
timing, rollback, capture, commit, and archive handling.

Descriptor storage is separate from callback context. The context is owned for
the full synchronous execution; no receipt or borrowed geometry escapes the
existing fixture callback. A diagnostic capture failure does not replace the
native rejection. Startup failures remain normal controller startup failures.

The initial host checks exercise dispatch, original ownership, error identity,
observation preservation, callback descriptor lifetime and exact cap/overflow
admission. They do not initialize physical owners or authenticate source setup.
The `setup()` forwarder is reviewed structurally, because constructing its
source requires the real startup factory. These checks do not prove physical
controller or archive equivalence; a subsequent observed production gate must
verify the real same-owner success/rejection and closed-prefix behavior. No
full-vehicle export mode is registered by this staged change.

With the existing live/original CMake configuration, build target
`robo_dyna_vehicle_run_observed_check`; focused CTest is
`vehicle_run_observed_values`. All build and GPU execution remains serialized
by the workstation guard. This authored change has not been compiled or run.
