# Current native-owner integration

The original implementation and numerical oracle below are reused unchanged
from 2c40dbbd. This successor applies them to 90a4a806 while preserving its newer
ordered assembly layout and cudaError_t assembly contract. Original frozen
numerical bodies remain unchanged. A separate `frozen/native_base` fixture and
`integration_proof.py` authenticate complete reversals of the four composed
storage/upload files to 90a. The budget oracle now uses that current layout;
its expected operand-only growth remains 785,640 device bytes. Fresh focused and
public-owner gates are required; old focused receipts are not new acceptance.

# Solid measurement operands

Baseline: TL `edab8e17e6d6e641f77f093fdb258d98152db492`. This bounded
slice moves only independent solid measurement operands into the existing five
parallel result-validation launches. The one-thread finalizer still applies
every floating `+=` in ascending family, parent, channel, and original local-slot
order. It does not change native element/material work, accepted force ownership,
status precedence, finite-check position, public readback, commit/rollback,
source admission, precision flags, or allocation caps.

## Storage and operation contract

`MeasurementOperands<Slots>` is fresh attempt scratch: four scalar doubles plus
separate kick/drift arrays (160 bytes for eight slots, 128 for six). The existing
fresh `result_valid` byte is its validity gate. Every validation row overwrites
both records, including failed status/history rows; failed rows do not read
unavailable parent, material, trial, accepted, or nodal-view inputs beyond the
unchanged predicate. Initial construction supplies no accepted/view reads.

For each valid noninitial row, `AccumulateMeasurementWork` records the exact old
accepted-RHS expressions separately:

1. `kick_dt * Dot(rhs, Mean(base_velocity, current_velocity))`
2. `Dot(rhs, Difference(current_position, base_position))`

No parent subtotal, slot deduplication, atomics, duration restriction, or early
derived-finite rejection is added. The final fold retains status before the flag,
native work/hourglass/plastic additions, strict native-dt minimum, each kick then
drift slot, and the original post-parent finite check. Thus partial diagnostics,
signed zero, overflow precedence, family short circuit, and retry freshness stay
observable at their original locations.

The count-only V5 shape adds 785,600 payload bytes and 40 `Storage` bytes, for an
exact tested device-layout delta of 785,640 bytes. `ArenaLayout` grows by 120 host
metadata bytes. The complete batch startup-host forecast is therefore expected
to grow by 785,800 bytes for otherwise identical input; root must confirm the
actual composed owner/application forecast rather than applying that expectation
to a concurrently changed tree.

## Proof and test inventory

`operand_proof.py` authenticates the complete five-file `edab8e1` baseline,
reverses current `Measure.h`, `Candidate.cu`, and `ResultValidation.cu` to exact
frozen bytes, proves the literal accepted-work extraction, and derives the new
fold from the old fold by counted substitutions. `verify_sources.py` checks the
owning manifest and composes the prior `solid_candidate_validation`,
`extended_solid_resident`, and `solid_law44_analytic` proof/donor chains. These
are source proofs, not numerical or CUDA execution.

The host target has eight functions: six exact value/failure/order/retry
comparisons against the complete frozen serial caller and two arena/forecast cap
functions. The CUDA target is root-only and contains nine functions: two prior
whole-caller/retry tests, three prior predicate/fault/grid-stride tests, and four
operand-path tests covering all-family faults/seeds/fresh retry, sensitive
accepted-RHS cancellation plus derived overflow, cross-family overflow
precedence, and more-than-worker failed-row overwrite. Private scratch bytes are
not compared to a nonexistent old scratch record; all histories, caches, status,
accepted slabs, and complete control diagnostics remain compared.

`check_syntax.py` compiles eight C++-shaped units with the real CUDA and Eigen
headers. It recursively expands the new wrapper, the prior wrapper, production
candidate, validation fixture, and generated frozen candidate before stripping
launch syntax. It never invokes NVCC or executes numerical code.

There is no qualifier `BUILD.bazel`; the preceding candidate qualifier likewise
has none. Production headers are already exported by
`//lib_src/elements/solids/resident:values`. The focused frozen CUDA package is
CMake-owned; do not advertise nonexistent Bazel test targets.

## Author-only light gates

Run each command through `tools/run_bounded.py` with `--cpus 1
--max-rss-gib 0.5`, a finite timeout, and fresh report/output paths. Do not reuse
an existing shape/build/report directory.

```sh
python3 -B lib_utest/qualification/solid_measurement_operands/verify_sources.py
python3 -B lib_utest/qualification/solid_measurement_operands/check_syntax.py \
  --output "$NEW_SHAPE"
cmake -S lib_utest/qualification/solid_measurement_operands -B "$NEW_BUILD" \
  -DCMAKE_BUILD_TYPE=Release
cmake --build "$NEW_BUILD" --parallel 1
ctest --test-dir "$NEW_BUILD" -V --output-junit "$NEW_HOST_XML" -j1
```

These author commands must not enable CUDA, invoke NVCC/native/Fortran, construct
the full source, or use a GPU. Keep `-fno-fast-math -ffp-contract=off`.

## Root qualification boundary

Only root/coordinator may run the hardware/native/full-owner gates under the
shared heavy lock:

```sh
cmake -S "$TL/lib_utest/qualification/solid_measurement_operands" -B "$BUILD" \
  -DCMAKE_BUILD_TYPE=Release -DTL_SOLID_MEASUREMENT_OPERANDS_CUDA=ON \
  -DCMAKE_CUDA_COMPILER=/usr/local/cuda/bin/nvcc \
  -DCMAKE_CUDA_ARCHITECTURES=120
cmake --build "$BUILD" --parallel 4
ctest --test-dir "$BUILD" -V --output-junit "$NEW_ROOT_XML"
```

CUDA keeps `--fmad=false --prec-div=true --prec-sqrt=true --ftz=false` and host
`-fno-fast-math -ffp-contract=off`. Root must then run affected
`solid_candidate_validation`, `extended_solid_resident`,
`solid_law44_analytic`, and legacy `solid_resident` CUDA/native/public-owner
gates in caches bound to the selected tree; verify owning Bazel
`resident:values`/`resident:batch`; reconcile exact full-V5 host/device forecast;
and perform the same-input accepted-archive comparison and ordinary throughput
measurement. A source/host/shape pass is not CUDA qualification or a speedup
claim.
