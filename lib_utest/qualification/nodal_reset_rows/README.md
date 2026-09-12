# Cooperative owner row reset

`FENodalState::BeginTrial` still submits one reset kernel on its own stream.
That kernel now uses one block of 256 threads. Lane zero preserves every original
Control clear, then executes the shared serial preflight. A uniform barrier
publishes its status to all lanes; rejection returns uniformly without touching
either row array. Successful lanes assign positive zero to distinct live entries,
then synchronize before lane zero publishes the original row metadata.

The actual owner allocates disjoint stiffness/damping ranges at `scratch+6*n` and
`scratch+7*n`. Counts and backing allocation are admitted at initialization.
There is no floating operation or reordered sum, no source/model special case,
no new kernel or CUDA call, and no allocation/layout/forecast growth. The earlier
`BeginTrial` scratch memset is unchanged, including its occurrence before a
reset rejection. Array-preservation claims for rejected resets describe the
reset kernel itself, not removal of that existing memset.

Public `stability::ResetRows` remains serial. `detail::BeginResetRows` extracts its
exact validation/invalidation prefix, and `CompleteResetRows` extracts its exact
metadata suffix. Other header fields remain untouched on invalid/stale reset.
The only production caller is the owner kernel; existing host and CUDA stability
tests remain valid. CIN's separately named `CinStorage::ResetTrial` is unchanged.

## Evidence and boundaries

Baseline `1d62620` is recorded in `baseline.json`. The complete original stability
header and complete original ResetTrial kernel are frozen. The oracle adapts
only the namespace, uses the identical RowBounds POD, and qualifies an unrelated
finalizer's `InvalidateRows` name to avoid ADL. Original reset statements remain
unchanged. `reset_proof.py` restores the full owner/header/BUILD bytes to their
baseline hashes and verifies the lane-zero/barrier/zero/publication body. The
existing seal proof consumes this checked reset-only header view; nested CIN
proofs and their prior source receipts remain active.

The composition on `1d3a441` retains the independently qualified CIN drift and
solid validation changes. Reset kernel/header bytes remain those of `68d3717`.
The BUILD proof removes the reset header entry, then invokes drift's checked
source-entry reversal to recover the common `1d62620` target exactly. Both frozen
trees remain unchanged. The composed source gate runs the full drift proof as
well as the limiter/seal chain; manifests retain both incoming digest histories.

The host gate compares the serial API with the complete frozen reset over
invalid pointers, equal pointers, count/capacity failures, stale epochs/attempts,
initialization state, precedence combinations, and retry. It checks positive-zero
bits and untouched guard/capacity entries containing NaNs, infinities and signed
zeros. A separate test checks the preflight/publication split.

CUDA qualification compares the actual private kernel with the complete old
kernel at counts 1, 255, 256, 257, 513, 65,537 and 1,048,579, including all header
rejections and subsequent successful reuse. Barriers have no lane-dependent
return path. Actual owner tests check discard/retry, rejected-reset header effects,
cleared assembly/limit/limiter witness, unchanged accepted fields and clock, later
commit, exact capacity and unchanged allocation count. Private test corruption
does not authorize mutating retained production views.

## Root gate

Under the existing root GPU/heavy-job guard, configure a fresh focused cache:

```sh
cmake -S "$TL/lib_utest/qualification/nodal_reset_rows" -B "$BUILD" \
  -DCMAKE_BUILD_TYPE=Release -DNODAL_RESET_ROWS_CUDA=ON \
  -DCMAKE_CUDA_COMPILER=/usr/local/cuda/bin/nvcc -DCMAKE_CUDA_ARCHITECTURES=120
cmake --build "$BUILD" --parallel 1
ctest --test-dir "$BUILD" --output-on-failure -j1
python3 -B "$TL/lib_utest/qualification/nodal_reset_rows/verify_sources.py"
```

The focused build includes the original `utest_step_stability.cc` and
`utest_step_stability_cuda.cu`, not rewritten copies. Also rebuild/run the existing
`nodal-seal-rows-root-1` and `cin-limiter-root-1` owning gates to cover accepted
limiter readback and downstream sealing, then the unchanged short V5 archive
comparison/timing. Bazel owning targets are `//lib_src/solvers:explicit_nodal_state`,
`//lib_utest/qualification/nodal_reset_rows:host` and `:cuda`; retain the two existing
`//lib_utest:utest_step_stability` and `:utest_step_stability_cuda` tests.

Author work is limited to tiny host execution, CUDA-shaped C++ syntax and source
proofs under 1 CPU/512 MiB. Actual CUDA barriers, runtime equivalence and throughput
remain root gates. No speedup is claimed from the scheduling change alone.
