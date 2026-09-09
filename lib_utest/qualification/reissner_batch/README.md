# P1 prescribed resident Q4 batch

The first bounded P1 gate passed on 2026-09-09: all five tests at 2 elements,
then the measurement/parity test in separate processes at 8, 32 and 128.
The immutable evidence summary is
`crash-work/reports/prescribed-batch-p1-summary-1.json` in the workspace.
`PrescribedShellBatch` calls the existing `ComputeShellForce` unchanged, using
one thread per element and 32 threads per block. It owns prescribed input and
trial output buffers, with no second nodal state, clock, assembly, dynamics or
material history. It must not be linked as a production accepted-state owner.

Only 2, 8, 32 and 128 elements are admitted. Every slot must be uploaded before
a launch. The lifetime limit is 100 evaluated attempts, including rejected
physical configurations; uploads cannot replenish it. CUDA failures poison the
instance. Per-element statuses are diagnostic, and successful lanes never
publish partial aggregate results. Caller output and timing remain untouched
on missing input, capacity errors, late element rejection or CUDA failure.

The POD sizes are guarded at compile time: reference 2664, elastic section
1176, configuration 224 and result 976 bytes. A single device arena owns
`count * (4064 + 976 + 4) + 8` bytes, or **645,640 bytes at 128 elements**.
The class has a preallocated host result/status staging area. Compile-time and
test checks keep the class, device arena, streamed host input and caller
result/snapshot arrays below 1 MiB. Device stream/events, module/stack backing,
other runtime allocations and actual CPU Chrono oracle objects are separate
resources; the workstation RSS/VRAM guard still applies. Inputs are streamed
one element at a time; there is no second full host input batch.

The external test composition is in
`robo-dyna/chrono/reissner_prescribed_batch_check.cu`, using the existing
`ReissnerShellCudaFixture.h`, actual Chrono `ReissnerReference`, and
`CopyReissnerShellSetup`. It needs an FEA-enabled Chrono build containing the
qualified coherent force path and `CH_REISSNER_CONSISTENT_FRAME_REFERENCE`.
The core helper has no Chrono or GTest dependency. Its owning CMake target is
`tl_prescribed_reissner_batch`; its opt-in, test-only Bazel target is
`//lib_utest/qualification/reissner_batch:prescribed_reissner_batch`.
`robo-dyna/chrono/PrescribedShellBatchChecks.cmake` composes the executable
`robo_dyna_reissner_prescribed_batch_check` and a serial 120-second CTest entry.
The external CMake composition is wired and built; this checkpoint does not
claim a Bazel build of the P1 helper.

Per-process environment selection is immutable:

- `ROBO_DYNA_P1_ELEMENTS`: 2 by default; explicitly 2/8/32/128 only.
- `ROBO_DYNA_P1_REPEATS`: 10 by default; explicitly 1 through 100.

The performance test is
`PrescribedShellBatch.SelectedResidentBatchMatchesActualChronoAndReportsMeasuredPhases`.
Run each requested size as a separate guarded process only after inspecting
the previous size's measured allocation/latency forecast. There is no automatic
size ladder. CPU oracle computation and result comparisons occur outside the timing intervals.
The other four tests exercise pre-allocation capacity admission, missing last
input/late failure/clean retry, a real 100-attempt limit, and safe invalid-launch
error poisoning. The limit test uses rejected unprepared references to avoid
100 expensive full physical evaluations; it is not a throughput measurement.

Every measured lane is a distinct prescribed, admitted configuration and is
checked against both actual CPU Chrono and the unchanged scalar TL host
operation with the existing `3e-12` dimensional parity tolerances. Representative
lanes also use the existing common-spin physical covariance gate; distinct
initial nodal frame parameterizations compare against the canonical reference.
The family at lane `i` is `i % 3`: canonical, initial frame offsets, changed
aspect. Their counts are respectively `(1,1,0)`, `(3,3,2)`, `(11,11,10)` and
`(43,43,42)` for the four admitted sizes. The 2-element run does not exercise
changed-aspect GPU lanes. These counts are derived from the pinned test source,
not additional XML measurements. This first
probe uses the existing fixture's fixed centered isotropic elastic layer,
thickness and density; it does not qualify source Yaris warpage, ELFORM, T3,
MAT024 or thickness-dependent vehicle throughput.

GTest XML properties retain all CUDA event/checked-host latency samples,
first/warm summaries, exact buffer bytes, `cudaMemGetInfo` after the initial
context query, allocation, kernel introspection, resident uploads, first/final
evaluation and destruction. Introspection may itself load a CUDA module.
Force events exclude aggregate reduction/readback; checked latency includes
those operations, completion, memory query and result publication. Stack limits
are queried and never changed. `cudaFuncGetAttributes.localSizeBytes` is not a
function-stack or local-traffic measurement. Compiler stack reports remain
separate. Occupancy API output is an estimate, not achieved occupancy; active
lane count, CUDA/runtime memory backing and device-wide guard deltas must not
be inferred from that estimate alone.

## Measured P1 checkpoint

Each process measured ten evaluations of the same resident inputs. Warm rows
below exclude the first evaluation, leaving nine samples. CPU Chrono and scalar
TL comparisons ran outside those intervals. The event span brackets the force
launch on its stream; it is not an activity-profiler kernel duration and can
include host enqueue or lazy-start gaps, particularly on the first launch.

| Elements | Warm force event median [range], ms | Warm checked median [range], ms | Explicit device bytes |
| --- | --- | --- | --- |
| 2 | 0.968960 [0.966784, 1.016416] | 0.993420 [0.992332, 1.047653] | 10,096 |
| 8 | 0.969024 [0.968256, 0.970464] | 0.995989 [0.994909, 0.998092] | 40,360 |
| 32 | 0.983040 [0.982464, 0.985120] | 1.010845 [1.010329, 1.014664] | 161,416 |
| 128 | 0.992576 [0.989248, 0.994048] | 1.041093 [1.038446, 1.042610] | 645,640 |

At 128, the separate explicit host-buffer forecast is 267,176 bytes, bringing
the reported buffer ledger to 912,816 bytes. Every size showed an additional
2,193,620,992 bytes (2.043 GiB) of device-wide use between the initial context
query and first completed evaluation: 4 MiB before force and 2,189,426,688 bytes
across the first force evaluation. Warm measurements were unchanged. These
samples do not attribute the difference to stack backing or the batch owner.

The compiler resource report records 255 registers and a 9,392-byte stack for
`EvaluatePrescribed`, and 25 registers/zero stack for status reduction. Its SHA256
is `e7d7957410f99c8f05f789d305ce24d6733c196fcd843e74054329f8f0d16229`.
The queried local size is also 9,392 bytes; neither source measures actual local
traffic. The occupancy API estimate is 1/6, but these launches contain only one
block at 2/8/32 elements and four blocks at 128 on a reported 170-SM device.
They cannot measure saturated occupancy or support extrapolation to a vehicle.

The four process guard reports passed with 2 CPU affinity slots, 2 GiB RSS,
4 GiB device-wide growth caps, and 32 GiB RAM/8 GiB VRAM reserves. Their short
processes yielded only two external samples apiece; sampled RSS/GPU maxima
missed the in-process allocation interval and must not be treated as true peaks.
The phase-specific `cudaMemGetInfo` samples are retained separately.

The original cap/runtime-poison assertions checked energy and one timing field;
the numerical late-failure test already checked complete bytes. A source-only
follow-up strengthens all three using shared, nonzero byte snapshots. Its
separate rebuild and 2-element rerun status belongs to the evidence summary;
it changes no force kernel or existing timing samples. P2 remains a separate
qualification-only scratch-layout experiment, with no production promotion.
