# Standalone native CPU/GPU batch measurement

This driver directly reuses the maintained native benchmark's `Cases.cpp` and
`Results.cpp` through the explicit `NATIVE_BENCHMARK_ROOT` CMake parameter. Pin
that checkout at `96a79464` and record its input files' hashes with each run.
The 256-pair corpus and all geometry provenance remain in that maintained
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
