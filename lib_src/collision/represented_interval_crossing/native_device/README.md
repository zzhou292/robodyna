# Standalone native CUDA certificates

This optional module executes the existing represented-linear native geometry
kernel on CUDA with the qualified shared eight-limb integer core. It is a
numerical executor, not a different contact profile, solver clock, contact law,
or source-identity scheme. Source authored; compilation, device parity and
performance are unqualified until the owning receipts say otherwise.

## Ownership and publication

`RepresentedIntervalCrossingGpu` owns the ordinary native frontend plus a
private `Workspace`. `DeviceExecution` is a private linkage seam so ordinary
CPU consumers remain CUDA-free. Its constructor is accessible only to the
concrete workspace. The frontend constructs a noncopyable `AuthenticatedWork`
borrow after authenticating the full source roster, canonicalizing pairs and
checking result capacity. No caller can pass an eligibility or sorted flag.

The workspace derives the fixed-integer domain from every actual pair. It
uploads only the bounded scene and compact admitted jobs. Each device worker
rechecks the original path keys and the same coordinate/depth domain before
running `CellKernel<512, FixedIntegerPolicy<512>>`. Other pairs remain with the
unchanged CPU implementation. They are routed before execution, not retried
after a device failure. Device arithmetic failure remains an explicit
`ExactArithmeticRange` result with the original work count.

Device output is staged by unique canonical ordinal; its completion markers
are checked before copying it into native staging. Existing CPU workers process
only incomplete rows. The unchanged native canonical fold applies total-work
limits and atomically publishes the complete result. The numerical kernel and
the normalization of every result field are shared, including unused fields.

The public facade takes its busy lease before touching workspace diagnostics.
No borrowed result is exposed while a call is running. All borrowed input
transfers are drained even if a later copy or launch fails. CUDA errors poison
the workspace and preserve prior native publication. No error silently falls
back to CPU. Inputs, native output and completion storage must be disjoint from
the entire workspace and facade. The owner uses one explicit stream/device.

## Exact arithmetic and portability

The public CPU kernel still uses its existing checked Boost policies. The
device instantiation reuses the qualified fixed integer primitive bodies and
the same dyadic, geometry and iterative traversal bodies. The lexical
arithmetic context resets only at pair entry. Every fixed operation latches
failure; existing fences after common translation and both cell evaluation
routes precede successful classification or another work increment. A faulted
normal is never published as ready.

The existing actual-input domain admits only the audited Boost1.74/64-bit limb
configuration and `B <= 125`, including zero coordinate exponents and the full
maximum sample depth. This bound covers every 512-bit intermediate and rounded
product. A device/host domain disagreement is a typed poisoned-owner failure.
No source IDs, approximate arithmetic, ambient rounding normalization or
compressed feature keys are introduced.

The 69-body extraction proof enumerates only approved policy adapters,
execution-space annotations and shipped standard-library equivalents. Both
ordinary C++ and nvcc host passes use `std`; only device compilation selects
`cuda::std`. The public record types are unchanged. No new bigint or manual
CUDA multiplication implementation is needed.

## Resources and qualification

One retained CUDA arena contains full represented paths, compact canonical
jobs, results, per-worker fixed scratch and per-worker DFS frames. One retained
host arena contains jobs and returned results. All aligned extents and owner
objects are included in `Preflight`; both CPU and extra workspace forecasts are
reported. Query calls do not allocate or resize retained storage. CUDA runtime
bookkeeping and compiler-managed device call frames remain subject to the
outer RSS/whole-device guard and actual compiler resource review.

The default remains 128 device workers in blocks of 32. An isolated capacity
extension admits 1 through 4096 workers for a guarded size/width benchmark;
every extra scratch and DFS row remains under the original explicit arena cap.
The default launches at most four blocks; the maximum can launch 128 blocks.
Each worker visits its grid-stride subset with unique output ownership. No
numerical work budget, frame, block size or publication rule changes with the
width. This is unselected experimental capacity, not a production throughput
claim. `--resource-usage` records registers, spills and call frames. No global
CUDA stack limit is changed. Qualification must record worker/block width,
scratch/DFS forecasts, actual device growth and compiler-reported local memory.

Owning tests compare complete native reports and result fields against CPU,
including canonical witnesses and physical work. They cover mixed admission,
wide/unsupported fallback, all-CPU and empty calls, source/identity rejection,
caps, wrong-stream rejection, alias rejection, retry and prior publication.
Existing CPU110 tests and frozen-body/source checks remain mandatory. Only a
warmed bounded benchmark with full parity may establish a component speedup.

## Compound adapter and integration boundary

`DeviceBatchAccess` reuses the existing native compound helper with its fixed
optional executor. It takes both owner leases for the whole cohort. Native
range/cap checks and an extra workspace-disjoint admission precede any borrowed
pair-key read. The complete roster authenticates once. After successful slice
admission, its lexical `PathRoster` materializes one noncopyable
`AuthenticatedScene` through a native-only construction key.

The first eligible slice uploads the scene. Later slices upload only their
compact jobs and return results. All-wide or empty cohorts upload no scene.
The only uploaded bit belongs to the native lexical scene and is bound to the
same executor, immutable path pointer and count. The workspace retains no input
pointer, scene token or generation cache. A drained successful operation marks
the scene uploaded; destruction of the native lexical object revokes this
authority automatically at outer return, including failure. Reusing an input
address on a later call requires authentication and upload again.

The existing native per-slice publication, work-cap prefix and last-successful
slice behavior remain in `BatchExecution.h`. Late failure keeps that native
publication and never advertises the outer caller scratch as complete.
Additional tests compare complete CPU/GPU batch reports and work at the
256/257 boundary, delayed and absent uploads, changed input/retry, malformed
unused paths, late work failure, aliases and canonical ordering/cap errors.

Raw and compound entry points are still **not** wired into transaction/app
code. The compound source is unqualified until its owning tests pass. Physics
acceptance and crash-video progress cannot be inferred from standalone
numerical parity or this module's benchmark.
