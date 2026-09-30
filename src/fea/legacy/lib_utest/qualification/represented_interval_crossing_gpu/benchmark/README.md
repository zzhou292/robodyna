# Standalone native CPU/GPU batch measurement

This driver directly reuses the maintained native benchmark's `Cases.cpp` and
`Results.cpp` through the explicit `NATIVE_BENCHMARK_ROOT` CMake parameter. Pin
the bounded-cohort helper checkout descended from `96a79464` and record its
source pin and input files' hashes with each run, including `PathFixture.h` and
`YarisGeometry.h`. The default 256-pair corpus and all geometry provenance remain in that maintained
module: source-derived Yaris geometry with synthetic independent identities,
plus small synthetic predicate families. This is not vehicle stepping.

One persistent CPU owner supplies the complete fieldwise oracle. Each measured
call is checked against it, including full native reports, witnesses, work, and
all published row fields. GPU mode also requires honest nonzero device routing;
out-of-domain rows retain their ordinary CPU fallback. Two warmups precede the
fixed repeat count. The timer encloses the complete public `Certify` call,
including host authentication/canonicalization, transfers, kernel execution,
synchronization, fallback, and publication. Corpus creation, owner/CUDA context
initialization and result verification are outside the timer. Raw GPU scene
upload is included on every call; no compound scene reuse is claimed here.

Build only after standalone parity and kernel resource admission pass. Run paired
CPU/GPU/GPU/CPU samples using the same explicit CPU affinity and GPU/RAM guards.
Both modes use four persistent CPU workers, including GPU fallback. Save every
sample, source/binary/compiler/flags pin, guard receipt and emitted result/work
digest. Compare semantic fields before reporting timing. Backend route/storage
fields legitimately differ; record them rather than presenting an all-device
claim. Do not extrapolate the small cohort's speed to full-vehicle throughput.

The bounded size/width sweep uses `--pairs 256|1024|4096` and
`--device-workers 128|512|2048|4096`. These are numerical component workloads,
not changes to physical transaction slice/work/publication capacities. Larger
cohorts retain the exact geometry and proportions of the original six families
with independent synthetic IDs. Device workers remain capped by the declared
owner forecast and actual job count. Retain each GPU benchmark process for more
than four seconds using the fixed repeat count: the guard polls GPU memory every
two seconds while its more frequent RSS rows reuse the last GPU sample.

`--view eligible` is an explicitly limited diagnostic view. It retains the full
source path roster and selects only pair/kind entries whose actual
`NativeStorageDomain` is eligible at the configured depth. No coordinate is
translated or rounded to force admission. The full original CPU result is
verified first, followed by an independent selected-pair CPU call and complete
GPU/CPU field checks. Output records original/selected/omitted pair and work
counts, both input digests, and actual device/host routes. This does not omit
difficult pairs in the production backend. Keep the mixed-cohort results visible;
an eligible-only benefit is not a vehicle or mixed-cohort speed claim.

## Compound publication comparison

`native_gpu_compound_benchmark` compares CPU compound calls, ordinary GPU calls
with `--numeric-cohort 0`, and optional numerical lookahead with
`--numeric-cohort 4096`. Every mode retains the original 256-pair native
input/result capacities and publication slices, 64 work visits per pair, and
16384 total native work limit. It uses the same full source roster and supports
the explicitly labeled mixed/eligible diagnostic views above.

A complete raw CPU result supplies the immutable row oracle. A real CPU
compound call supplies the complete compound report and last native report;
no aggregate native report is fabricated. Every timed iteration checks all
compound scalar fields, full native report fields, all outer result rows, the
last published slice and exact path-authentication operation counts. Device
checks distinguish submitted and consumed rows, launches, uploads, numerical
cohorts and fault metadata. Consumed means native staging, not physical commit.

The timer wraps the entire compound call, including authentication, upload,
numerical preparation, readback, synchronization, CPU fallback and every original
slice publication. Initialization, full source/selected CPU oracle construction
and fieldwise verification are outside timing. Default benchmark worker width
is explicitly 4096; production defaults remain 128 and cohort lookahead stays
disabled. Retain source/toolchain/guard pins and compare CPU/GPU semantic fields
before interpreting timings. This benchmark does not select a vehicle backend.
