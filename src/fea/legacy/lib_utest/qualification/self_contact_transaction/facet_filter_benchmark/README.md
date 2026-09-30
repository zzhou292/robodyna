# Facet-filter adapter query benchmark

Build this same driver against the old adapter (`3c0ee611`) and the frozen compact
adapter through `TL_FILTER_ROOT`. `--backend scalar` invokes the existing public
prism certificate; `--backend adapter` invokes the actual selected `FacetFilters`
implementation. The only compatibility helper adapts the old void-returning
`BeginCandidateChunk` signature to the new report-returning signature. No old-span
or compact scheduling algorithm is duplicated in this driver.

The small authenticated binding/geometry fixture is reused from qualification
code. GTest is linked **only into this dedicated benchmark** so that setup is not
copied into a new fixture. All fixture assertions, source/material/facet readiness,
geometry preparation, owner allocation and scene upload are checked before
timing. Product CLI dependencies and source are unchanged.

The synthetic patterns are `all_linear`, `alternating`, `short_islands` (eight
linear rows followed by two nonqueries), and `no_linear`. Pair count is explicitly
bounded at 4096. They reuse the same accepted/prepared fixture coordinates and
complete parent thicknesses. Motion routing declarations are synthetic: the
benchmark does not execute nonlinear proofs, establish rigid exclusions for a
vehicle, or claim a source-authenticated Yaris distribution. Its purpose is to
measure query fragmentation and packing on identical numerical inputs.

Each iteration includes beginning the chunk, serial motion classification, all
prism calls or GPU transfers/kernels/synchronization, and storing results in
original order. Every row's action and full filter result are compared outside
the timer against the public scalar reference. Two warmups precede a fixed
repeat count. No per-query allocation is introduced by the driver.

A scoped linker wrapper forwards every real `cudaMemcpyAsync` call unchanged.
Observation starts **after scene upload**, so H2D bytes describe actual pair
queries. It records call count, bytes and minimum/maximum pair cardinality;
readback counts/bytes must match exactly. Scalar and no-linear modes must issue
no query copies. Every linear row must be transferred exactly once per iteration.
The wrapper's small observation cost is included in both adapter measurements;
it is not present in the product. Timer-stage counts are not used as query counts.

Example commands, executed only by the guarded heavy lane after owning tests:

```text
cmake -S <this-directory> -B <separate-build> -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_CUDA_ARCHITECTURES=120 -DTL_FILTER_ROOT=<pinned-TL-tree>
cmake --build <separate-build> --target facet_filter_adapter_benchmark -j4
<binary> --backend adapter --pattern alternating --pairs 4096 --repeats 100
<binary> --backend scalar --pattern alternating --pairs 4096 --repeats 100
```

Use identical affinity/resource guards and alternating old/new/old controls.
Save complete JSON, source/compiler/binary pins and guard receipts. Compare input
and result digests, route counts and transferred linear rows before timing.
Pin the selected current-regularity, active-use, QBAT catalog/binding fixture
headers and filter `Corpus.h` as well; they are intentionally read from
`TL_FILTER_ROOT`, not copied into this driver.
Adapter query counts legitimately change. Scene upload is reported as setup,
not hidden in the query cost. A component gain is not a vehicle-speed claim;
the next real probe must retain exact state/receipt checks and actual query counts.
