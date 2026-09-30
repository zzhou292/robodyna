# Observed controller qualification

This internal diagnostic composition lets an existing `PreparedRun` freeze a native
self-contact rejection during its ordinary execution. It does not introduce a
solver, owner, clock, alternate run loop, or global/environment solver setting.
The ordinary CLI and `PreparedRun::Execute` use the unchanged null-hook path.
The explicit CLI failure-output option uses the narrow
[diagnostics facade](../../diagnostics/README.md); its public interface exposes no
callback or publication authority.

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
verify the real same-owner success/rejection and closed-prefix behavior.

With the existing live/original CMake configuration, build target
`robo_dyna_vehicle_run_observed_check`; focused CTest is
`vehicle_run_observed_values`. All build and GPU execution remains serialized
by the workstation guard. The integrated app, including the observer and full
gate wrapper, has compiled successfully against native raw-crossing batching
(`crossing-batch-app-build-1`). Its six observer host tests passed. The first
nine-CTest invocation passed eight checks; native replay stopped before work
because its SHA environment variable was misspelled. The corrected native-only
replay passed (`crossing-batch-app-native-replay-1`). Preserve the initial failed
receipt `crossing-batch-app-host-tests-1`; it is not a numerical replay failure.
The full observed two-interval controller gate remains pending. These build and
host checks do not establish real observed-controller physics or archive
qualification.


The original and observed full gates share `tests/TwoIntervalAcceptance.cpp`.
Its source factory, 1 micrometre declared wall gap, 200 ns step, two-commit
requirements and archive/ledger checks are identical; only execution dispatch
varies. The ordinary wrapper calls `PreparedRun::Execute`. The observed wrapper
calls `RunAccess::Execute`, publishes a captured rejection after normal closure,
and returns the unmodified controller result to the same assertions. A native
rejection therefore FAILS this acceptance test even if diagnostic export works.
There is no expected-failure registration or success-on-capture path.

With the existing `ROBO_DYNA_ENABLE_V5_SELF_CONTACT_CONTROLLER` option, the extra
CTest is `vehicle_run_wall_self_contact_observed_two_intervals`, in the existing
`robo_dyna_vehicle_run_original_check` executable. It requires two explicit
inherited environment paths before source/owner construction:

- `ROBO_VEHICLE_RUN_OUTPUT`: a pre-created empty real controller directory.
- `ROBO_SELF_CONTACT_FAILURE_OUTPUT`: an absent directory with an existing real
  parent, outside the controller directory. It remains absent if no pair is
  captured; existing or partially exported evidence must never be overwritten.

A captured pair is exported with `CandidateFailureFixture::Export`, which reads
back its bounded native codec and returns the manifest SHA. The test records
that hash/path and compares the captured typed report with the normal controller
rejection. A subsequent bounded host call to `ReplayCandidateFailure` uses this
explicit hash; its standalone classification is diagnostic, never acceptance.
Capture/export failures are additional test failures and cannot replace the
controller rejection or bypass the shared commit/archive assertions.

The current observer captures authenticated prepared-intersection and final
candidate-policy failures. Native raw-crossing failures also reach the observer
when their pair ordinal maps to an authenticated source facet pair. The native
transaction maps the subbatch ordinal through the complete canonical roster
before rollback; its report retains the native admitted work prefix, triggering
pair work and limit when those fields are available. A work-limit failure is
resource evidence, not a geometric rejection witness. Failures without an
authenticated pair can still produce no fixture and still fail acceptance.
No native proof limits are changed by this wrapper.

Caller-pinned replay requires `ROBO_SELF_CONTACT_FAILURE_MANIFEST` and
`ROBO_SELF_CONTACT_FAILURE_SHA256`. The latter is the exact manifest SHA returned
by export; `ROBO_SELF_CONTACT_FAILURE_MANIFEST_SHA256` is not a recognized name.
