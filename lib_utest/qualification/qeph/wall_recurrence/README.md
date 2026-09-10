# CW1 native recurrence support

Source preparation only; no execution or contact-screen admission is claimed.
The prospective physical tuple, six-step grid, three amplitudes, full-state
metric and numerical budgets are frozen in
`planning/QEPH_WALL_RECURRENCE_SCREEN.md`. These helpers do not implement that
whole screen or alter the qualified BQ4 free-response policy.

`MovingNativeProbe` retains the full 109/194-coordinate matrix and actual moving
baseline. The shared `NativeMapWithUniformVelocity` applies a physical common
velocity before normalized perturbations, then runs the existing history/cache,
kick, drift and native Q2 chain. Its output stays relative to reference/rest:
uniform velocity and its drift remain observable. A zero boost follows the old
arithmetic; the existing `NativeMap` and `Differentiate` contracts remain intact.
No startup admission, material owner or contact force has been added.

`free_response/RecurrencePowerGram` exposes the existing binary loop's final
power and pre-transition Gram. Its chronological composition is
`P=Pb*Pa`, `G=Ga+Pa.transpose()*Gb*Pa`, for a followed by b. A separate helper
adds the final endpoint once. Zero transitions are supported by the new API;
the legacy `PowerGram` still rejects count zero. No operator is symmetrized or
reduced here, and these helpers make no stability decision.

Prospective host-test budgets are fixed before execution: analytic matrix and
native normalized-field comparisons use `2e-12 * max(1, abs(expected))`, the
existing analytic helper scale; matrix-column control separation uses the
frozen `5e-8` matrix budget with a factor of 32. Legacy-wrapper parity and
failure publication checks are exact. The tests include moving reference/rest
baselines at all six h values and V=-8,0,+8 m/s, but are not a substitute for
complete wall branch, amplitude, metric or switching-product qualification.

Parent wiring needs `RecurrencePowerGram.cpp` added to the existing
`qeph_recurrence_audit_core`, and a host test target containing
`MovingNativeProbe.cpp`, `MovingNativeProbeTest.cpp` and
`PowerGramCompositionTest.cpp`, linked to that core and `GTest::gtest_main`.
Use C++17, `EIGEN_DONT_PARALLELIZE`, `-fno-fast-math -ffp-contract=off`, one CPU
and the existing 60-second qualification timeout. No CUDA dependency is added.
The old audit/response regressions must pass before reuse; no checkpoint or
source map is changed by this source-only increment.

The prepared test suite contains four `QephWallRecurrenceSupport` functions and
six `QephWallPowerGram` functions. No full contact matrix, active-set schedule,
gain threshold or report acceptance is implemented by this support increment.
