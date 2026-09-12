# Optional CUDA kernel event timing

This separate Linux startup `LD_PRELOAD` diagnostic measures same-stream CUDA
event intervals around outer kernel launches. It serializes intercepted launch
wrappers and synchronizes each end event. It changes scheduling and execution
overlap; its elapsed process time is not ordinary solver throughput. No solver,
state, physics, clock, archive, or ordinary API-timing code is modified.

The existing `benchmarks/cuda_api_timing` supplied the standalone C++/fake-runtime
build, one-time `RTLD_NEXT` resolution, startup TLS, exact forwarding and
create-only report patterns. Its API wall-time counters include waited-for work
and cannot measure device kernels. This diagnostic uses event intervals instead.

## Verified launch paths

The current vehicle executable dynamically imports `libcudart.so.13`,
`__cudaRegisterFunction`, `__cudaGetKernel`, and `__cudaLaunchKernel`. CUDA 13
`crt/device_functions.h` declares the internal launch's first argument as
`cudaKernel_t`; `crt/host_runtime.h` obtains that handle from the registered host
function using `__cudaGetKernel` before launching. This is distinct from the
public `cudaLaunchKernel(const void*, ...)` entry-function pointer API.

Both launches are intercepted. Registered names are copied into a fixed table;
only successful handle lookups bind a runtime handle to that registration.
Names are never guessed from handle values. Missing registration/lookup, retired
modules, and table overflow yield an explicit unknown bucket. Module
unregistration invalidates its mappings, while prior counters/names remain.
Handle rebinding requires a new successful lookup. Truncated names are marked;
registration index distinguishes separate registrations with equal names.

Run the read-only ELF check before attribution:

```sh
python3 -B benchmarks/cuda_kernel_timing/inspect_imports.py /absolute/path/to/executable
```

It accepts the two verified paths and rejects direct graph, driver, extended,
cooperative, or per-thread-default-stream launch imports. It only inspects the
named ELF; repeat for application DSOs owning kernels. Dynamic plugins, static
runtime binding, driver/device-side launches and unobserved entrypoints are not
covered. A passing direct-import check alone does not prove complete coverage.
Inspect nonzero observed launch counts and unknown/drop counters in the report.

## Build and CPU qualification

```sh
cmake -S benchmarks/cuda_kernel_timing -B /absolute/new/kernel-timing-build \
  -DCMAKE_BUILD_TYPE=Release -DROBO_DYNA_CUDA_KERNEL_TIMING_TESTS=ON
cmake --build /absolute/new/kernel-timing-build -j1
ctest --test-dir /absolute/new/kernel-timing-build --output-on-failure
```

This C++17 target needs CUDA 13 headers, `dl`, and pthreads. It links no CUDA
runtime and runs no kernels during compilation/tests. Tests use a CPU-only fake
runtime with the real `libcudart.so.13` symbol version/soname. They cover both
launch paths, pointer/dimensions/stream/argument forwarding, return/errno,
successful-only handle binding, unregister/re-register, early registration,
concurrency/nesting, bounded tables/names, simulated current-device changes,
capture, each event API failure, invalid elapsed times and create-only output.
Keep this build directory out of production `LD_LIBRARY_PATH`: it contains a
fake `libcudart.so.13` solely for tests. The profiler `.so` does not need it.

## Explicit diagnostic invocation

Use the existing workstation guard and unchanged short-run arguments. Set the
following variables only on the simulation child, with a new report filename:

```sh
env LD_PRELOAD=/absolute/kernel-timing-build/librobo_dyna_cuda_kernel_timing.so \
  ROBO_DYNA_CUDA_KERNEL_TIMING_OUTPUT=/absolute/new/kernel-events.json \
  /absolute/path/to/robo_dyna_vehicle_run [same qualified short-run arguments]
```

Restore ordinary execution by omitting these two environment assignments; no
rebuild or source restoration is needed. A loaded shim with absent/empty output
environment forwards without CUDA instrumentation. This is not late `dlopen`
injection. Original launch arguments and its actual returned result/errno are
forwarded exactly once, including when instrumentation fails. No
`cudaGetLastError` is called. Instrumentation can expose asynchronous errors
earlier; any instrumentation failure invalidates timing attribution and requires
investigation, not suppression of the solver's errors.

Before events, `cudaStreamIsCapturing` detects active/invalidated graph capture;
these launches are forwarded unmeasured without injecting events. Failure to
query capture also leaves the original launch unmeasured. A begin event precedes
the launch in its own stream; an end event follows and is synchronized. The
event timestamps exclude earlier queued work in that stream, but their region
may include host submission gaps, same-stream work submitted by other threads,
or effects of other streams/processes. Tiny kernels should not be interpreted
as instruction-time measurements. Startup and every later launch are counted;
there is no application-specific epoch or zero-motion shortcut.

The outer measurement mutex permits only one temporary event pair, created and
destroyed within the calling thread's current device context. No event pool,
device switching, per-launch retained record, or solver-state readback is added.
Runtime-internal nested launches are forwarded unmeasured to avoid double
counting. Normal application synchronization remains untouched.

## Report and bounds

`robo_dyna.cuda_kernel_timing.v1` reports each registration's total/failed/timed
launches; total, maximum and first timed device nanoseconds; first grid/block
and shared bytes; and changed-dimension count. CUDA returns event time as float
milliseconds; conversion rounds that observation to nanoseconds and does not
claim nanosecond measurement precision. First timed launch is not a declared
physical TT0 phase. Use counts and totals with the application stage timing and
an explicit short-run window.

The report includes unknown launches, dropped registration/handle counts,
truncated names, capture/nested skips, instrumentation errors, invalid times,
counter saturation and active calls at shutdown. Counters saturate rather than
wrap. There are 2,048 immutable registration slots with 1,024 bytes/name and
4,096 handle slots; `fixed_state_bytes` reports the exact compiled state size
(under 4 MiB). Tables never grow. Output is capped at 16 MiB using a 1 KiB
format buffer; paths are capped at 4,095 bytes. CUDA owns implementation-specific
memory for the at-most-two transient events; this diagnostic does not claim a
known CUDA event allocation size.

At normal shutdown a bounded coherent CPU snapshot is written with
`O_EXCL|O_NOFOLLOW`, mode 0600. Existing files/symlinks remain intact. No CUDA
calls or waits occur during shutdown. A nonzero active count means the snapshot
is incomplete. A failed write may leave a partial file and emits a warning;
signals, `_exit`, or crashes need not produce a report. Only parse a complete
JSON result. Registration and timing structures remain process-lifetime storage.

Root qualification must first check the real executable's imports, then run a
short actual V5 profile and compare the entire archive/raw native result with
the unchanged baseline. Require zero unknown launches, dropped mappings,
instrumentation failures, invalid times, saturation and active shutdown calls
before attributing observed kernel costs. Profiler output is diagnostic evidence,
not a physics qualification or an optimization by itself.
