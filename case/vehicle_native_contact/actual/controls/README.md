# Native preview cooperative controls

This small adapter configures the existing vehicle `Control` and `RunLoop`.
It does not add a solver clock, driver, physical limit, timestep policy or signal
handler. Defaults preserve the current unbounded preview behavior.

Only the explicit native preview forecast/run reads these optional variables:

- `ROBO_NATIVE_VEHICLE_MAXIMUM_ELAPSED_S`: finite nonnegative seconds. Zero or
  absence disables the limit. This is the existing RunLoop elapsed clock: it
  excludes source preparation, owner startup and retry qualification. The limit
  is checked between accepted intervals, not inside a CUDA kernel.
- `ROBO_NATIVE_VEHICLE_STOP_FILE`: a nonempty path, at most 4096 characters.
  Prefer an absolute initially absent path outside the archive. Create the file
  to request a stop. The adapter follows the existing vehicle CLI `exists`
  semantics and never creates, consumes or removes this file. A preexisting
  file requests an immediate stop after initial accepted capture.

Malformed explicit settings fail before source construction or owner creation.
The effective options are written to the qualification `case.json`. Ordinary
owner qualification ignores both preview options and retains its strict retry
and two-interval checks. These variables do not change requested duration,
samples, source identity, equations or failure behavior.

At a cooperative boundary the existing loop records the actual last accepted
state and calls `FinishPrefix`. The summary continues to report an incomplete
horizon and the specific requested/time stop. The full-horizon actual GTest
continues to fail if it is stopped early: a valid diagnostic prefix is not a
passing collision delivery gate. Read the guard's child exit and the archive
manifest separately; direct Chrono replay may inspect a valid closed prefix.
No physical restart is created.

Choose the cooperative limit sufficiently below the external guard timeout to
allow startup, the current step, final capture, archive closure and replay.
The external resource guard remains authoritative and may kill the process
before cooperative closure. No graceful-closure guarantee applies to a guard
kill, allocation failure, capture failure or failed archive write.

Focused host qualification reuses the actual `RunLoop`, existing vehicle value
library, and all original `LoopTest.cpp` regressions. New tests cover unset/zero
defaults, invalid input, elapsed stop at an off-cadence accepted endpoint, file
polling and preservation, and an initially requested stop. Tests use fake owner
operations to verify final accepted sampling and prefix finalization; they are
not a real-vehicle runtime or on-disk archive qualification.

```sh
cmake -S case/vehicle_native_contact/actual/controls/tests -B /absolute/fresh/build
cmake --build /absolute/fresh/build -j4
ctest --test-dir /absolute/fresh/build --output-on-failure
```

Build and invoke the real actual wrapper in a separate guarded integration gate
before replacing the qualified vehicle executable. Preserve the currently
running unmodified executable and source checkouts throughout qualification.
