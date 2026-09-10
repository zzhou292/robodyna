# Optional CUDA runtime API timing

This Linux startup `LD_PRELOAD` library measures the wall time of four public
CUDA runtime calls: `cudaMemcpyAsync`, `cudaStreamSynchronize`, `cudaMemcpy`, and
`cudaDeviceSynchronize`. It forwards the original pointers, sizes, direction,
stream, return value, and `errno`. It changes no streams or physics and performs
no CUDA queries or extra synchronization.

Build separately from the solver; this needs CUDA headers, a C++17 compiler,
`dl` and `pthread`, but the library does not link or initialize CUDA:

```sh
cmake -S benchmarks/cuda_api_timing -B /tmp/robo-dyna-cuda-timing -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/robo-dyna-cuda-timing -j1
ctest --test-dir /tmp/robo-dyna-cuda-timing --output-on-failure
```

The test runtime has the `libcudart.so.13` soname and uses CPU buffers only.
It verifies exact forwarding and failures, nested calls, concurrent counters,
directional bytes, disabled collection, and create-only output rejection.
Keep its build directory out of production `LD_LIBRARY_PATH`.

For a separately authorized, guarded simulation command, set these variables
on the simulation child only (replace the report path with a new filename):

```sh
env LD_PRELOAD=/tmp/robo-dyna-cuda-timing/librobo_dyna_cuda_api_timing.so \
    ROBO_DYNA_CUDA_TIMING_OUTPUT=/absolute/new/timing.json \
    /absolute/path/to/robo_dyna_source_assembly_wall [existing arguments]
```

With the output variable absent or empty, wrappers forward without collecting
timestamps or counters. Real symbols are resolved once during library startup.
The normal wrappers use fixed lock-free counters and startup TLS;
they allocate nothing themselves. `BIND_NOW` avoids their first-call PLT lazy
binding. Real CUDA operations retain whatever allocations their runtime needs.
This is a startup preload, not a supported late `dlopen` injection.
If a dependency DSO constructor calls CUDA first, its wrapper invokes the same
one-time resolver before forwarding. Only this resolution may allocate through
`dlsym`; it is outside measured/counted intervals, preserves caller `errno`,
and also works when collection is disabled. The fake runtime tests both modes.
Calls before the timing library's constructor are forwarded but uncounted.
A loader callback that recursively requests an as-yet-unresolved CUDA function
cannot be forwarded: a TLS guard emits a distinct message and exits 127 rather
than deadlock in the resolver. Already resolved nested symbols can forward.

The `robo_dyna.cuda_api_timing.v1` JSON report contains:

- `process_interval_ns`: monotonic elapsed time from library startup to report
  preparation at normal process shutdown, including startup and output work.
- Per-function completed outer calls, failure count, total and maximum API wall
  nanoseconds, requested and successful bytes by declared CUDA direction.
- Nested calls omitted from timing to avoid counting their time twice, calls
  still active at shutdown, missing symbols, clock failures and saturation.

API wall time includes work waited for inside that call. It is **not device
kernel time**; asynchronous calls may include host staging and runtime work.
Concurrent thread intervals can overlap, so their sum need not fit the process
interval. `runtime_default` bytes remain unresolved; no pointer query is added.
Successful asynchronous bytes mean the API accepted the request, not that its
device work completed successfully. Only outer calls receive per-API counts;
the report separately counts nested calls that were forwarded.

Only these normal public dynamically linked symbols are covered. Driver APIs,
static CUDA runtime linkage and per-thread-default-stream suffixed symbols are
outside scope. A missing next symbol is reported at shutdown if unused; if
called, the library exits 127 rather than fabricate a CUDA return value.

At normal shutdown the report is formatted outside wrappers into at most 64 KiB
and created with `O_EXCL | O_NOFOLLOW`, mode 0600. Existing files and symlinks are
preserved. Report failure produces a bounded stderr message and does not change
the application's exit status; an interrupted write may leave a partial file.
No directories are created. Paths must fit 4095 bytes. `_exit`, termination by
signal and crashes need not emit a report. A nonzero `active_calls_at_shutdown`
marks a concurrent/incomplete sample; shutdown never waits for worker threads.

Compare the simulation's accepted endpoint and archive against its qualified
reference before interpreting timings. This utility diagnoses where host API
time goes; it does not establish simulation correctness or justify a longer run.
