# Bounded elastic coupon profile (P0)

`robo-dyna-coupon-profile` profiles the existing `ElasticCouponCase` public API.
It owns one case and runs a short prefix of the two-Q4 elastic release. The
case's material, timestep, force operation, validation receipt and numerical
thresholds remain unchanged. This is performance evidence, not an accepted
trajectory artifact bundle or a new physics qualification.

The default is 10 warmup steps followed by three consecutive windows of 100
measured accepted steps. `--warmup` accepts 1..100, `--steps-per-repeat` 1..300,
and `--repetitions` 1..5. Their combined count must not exceed 1000 and must
remain below the case's full admitted horizon. Counts are fixed before launch;
the harness never extends an experiment to improve its timing result.

The profiling configuration uses `diagnostic_intervals=64`, explicitly separate
from the accepted B2 run's cadence of 20. With the current 18,830-step horizon,
the derived audit stride is 295; the default 310-step profile therefore includes
an actual measured spectral audit. Results classify each successful step by the
observed change in the case's audit counter. A shorter custom request may report
`not_observed` for audited steps; that is not a zero-cost measurement.

## Build integration

Sources are split by responsibility: `CouponProfile.cpp` runs the bounded case,
`CouponProfileReport.cpp` formats telemetry, and `coupon_profile_main.cpp` parses
the CLI and publishes the create-only result. `CouponProfileData.h` is private
telemetry. The public header is `CouponProfile.h`.

Enable `ROBO_DYNA_ENABLE_ELASTIC_COUPON` and `ROBO_DYNA_ENABLE_COUPON_PROFILE` in
the existing external CMake project. Its `robo-dyna-coupon-profile` target links
the existing elastic coupon case, `robo_dyna_artifact_io` and `CUDA::cudart`.
The first bounded 310-step run passes; results are in
[`coupon-profile-p0-result-1.json`](../../crash-work/reports/coupon-profile-p0-result-1.json).

## Guarded invocation

Run from the workspace root only when the shared build/GPU slot is free. Both
paths must be new, and their parent directory must already exist. The command
below assumes the orchestration build exposes the target under `benchmarks/`:

```bash
python3 Total-Lagrangian-FEA/tools/run_bounded.py \
  --report crash-work/reports/coupon-profile-p0-1.json \
  --lock crash-work/reports/workstation.lock \
  --cpus 2 --min-available-gib 32 --max-rss-gib 1 \
  --timeout 60 --gpu 0 --min-gpu-free-gib 8 --max-gpu-growth-gib 4 \
  -- crash-work/build/elastic-coupon/benchmarks/robo-dyna-coupon-profile \
     crash-work/reports/coupon-profile-p0-result-1.json
```

The guard owns CPU affinity, numerical worker limits, timeout and sampled
process/GPU resource enforcement. The harness does not override CUDA stack
limits or reset the device. Record the executable build/source revision with
the guard result; compare runs only after checking the same configuration and
unchanged admitted numerical envelope. A successful program returns zero after
writing its result. Invalid arguments, API failures or rejected steps return
nonzero without publishing a successful result. Shared artifact writes are
create-only under external serialization, not an atomic filesystem transaction;
an I/O failure can leave an incomplete file that must not be used as evidence.

## Reading the measurements

- `cold_phases` separates the first CUDA call, empty case construction, complete
  case initialization and destruction. There is one cold lifecycle. Initialize
  includes CPU modal qualification, allocation, first assembly and initial mesh
  publication; the public API cannot split those operations further.
- `step_samples` retains at most 1000 accepted epoch/time/timing records.
  `timings` reports count, minimum, maximum, median, mean and sum for warmup,
  measured ordinary steps, audit-bearing steps, each repetition and captures.
  Warmup is excluded from measured groups. Consecutive repetitions sample one
  evolving trajectory; they are not independent reinitializations.
- Every `Step` time includes the case's existing assembly, integration, candidate
  checks, synchronization and commit. An audit-bearing time additionally includes
  full-state copying and CPU spectral audit. Neither is a kernel-only timer;
  subtracting one from the other would not isolate CPU or device work.
- `capture_samples` times an initial accepted capture and one after each measured
  repetition. Capture reads accepted fields and publishes the Chrono mesh. Disk
  archives, JSON result serialization and VSG rendering are outside that timer.
- `memory_samples` uses `cudaMemGetInfo` between phases, with query time separate
  from step timing. Device-wide free/used bytes include runtime and other users;
  explicitly owned state/element bytes come separately from their allocation
  ledgers. Sparse samples are not peaks and cannot attribute growth to kernels,
  stack backing or this process. The external guard supplies a pre-process GPU
  baseline: querying CUDA memory inside the process before the first-call timer
  would contaminate initialization. Case destruction may leave runtime memory.
- `final_accepted_metrics` records actual epoch/time, energy components, observed
  energy error, geometry/work diagnostics and last spectral audit. Admission
  constants and sampled operator bound are saved alongside the measurements.
  The run must still pass every unchanged case gate. No timing assertions or
  extrapolated vehicle throughput are part of this profile.

P0 now has public-API measurements; internal kernel/event counters and a
representative prescribed batch remain next. Consult
[`YARIS_SHELL_SCALING_REVIEW.md`](../../planning/YARIS_SHELL_SCALING_REVIEW.md)
for the separate P1/P2/P3 qualification boundaries.
