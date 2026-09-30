# Optional case stage timing

`StageTimer<N>` is a fixed-storage host diagnostic for a serialized coordinator.
It measures existing callables and preserves their result, exception and errno.
An enabled timer reads a per-instance monotonic clock before/after a call. A
disabled timer returns the callable directly before reading any clock or updating
any counter. Neither path allocates or calls CUDA. Tests inject an independent
clock/context into each timer; production uses `CLOCK_MONOTONIC`.

Slot zero records an inclusive initialized case `Step()` call, including calls
rejected before an owner trial exists. Other slots record disjoint component
calls. They are call counts, not owner attempts or accepted physics epochs.
`last_step` is reset for each initialized call; totals retain rejected calls and
their failed stages. Clock failures and backward samples do not contribute a
duration. Counters saturate explicitly instead of wrapping. Clock errors never
change a mechanics return or decide whether a trial commits.

The case owns one timer inline, with no heap/device allocation or change to the
TL owner. On the qualified x86-64 GCC 11 layout, `sizeof(StepTimer)` is 1,960
bytes; existing padding makes `sizeof(Impl)` grow from 5,600 to 7,552 bytes
(+1,952). Both enabled and disabled cases include this fixed payload in the
existing `sizeof(Impl)` startup host-budget check. There is no per-step growth.
The physical Config, accepted Diagnostics, stamps, archive schemas, and stream
operations are unchanged. The public snapshot is a copied diagnostic value.

Wall time includes waits already performed by the measured operation. An
asynchronous launch may return quickly and charge its queued work to a later
readback. These measurements identify expensive call boundaries; they are not
device kernel durations. Inclusive Step time also includes unmeasured host
bookkeeping and timer overhead, so it is reported separately from the sum of
component durations. Initialization, accepted output capture and archive I/O are
outside this timer.

Standalone host qualification (no Chrono, TL runtime or CUDA dependency):

```sh
cmake -S benchmarks/stage_timing -B /tmp/robo-dyna-stage-timing -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/robo-dyna-stage-timing --parallel 1
ctest --test-dir /tmp/robo-dyna-stage-timing --output-on-failure
```

The assembly dynamics owning CUDA target additionally includes
`OptionalTimingPreservesActualFieldsHistoriesAndAllocationAcrossContact`: two
actual cases run 64 intervals with timing off/on, checking copied fields,
histories, contact/work samples, allocations and complete stage counts. Full
archive parity and profiling remain separate integration checks.
