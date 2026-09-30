# General parallel CIN dependent drift

The existing admitted CIN source proves unique secondary nodes and excludes
secondaries from every master set. Each dependent drift reads only that node's
accepted position/quaternion and recovered trial velocity/angular velocity.
The optimization uses this contract for all admitted sources and arbitrary
source-row/domain-node ordering, loads, rotations and geometry. It contains no
vehicle-count assumption, zero-motion shortcut or floating-point reduction.

After **every** recovery row succeeds, `Drift.cu` resets the now-dead recovery
failure key and computes one fresh `drift::Row` per source row. Integer `atomicMin`
selects the earliest **source row**, not the smallest node. Publication writes
all earlier rows and exactly the original failing-axis position/reaction prefix
of the failed row. Orientation failure follows all three position/reaction
stores and leaves the quaternion unchanged. Quaternion stores happen only on
success. `StepTooLarge` still sets the existing limit to zero. Later rows and
all accepted state remain untouched. A recovery or earlier-stage failure gates
every drift kernel and the subsequent unchanged capture phase.

The neutral orientation helper preserves the original increment/hypot/angle
check before reading the quaternion, and preserves the complete native
quaternion expressions. Its existing in-place wrapper keeps all four stores.
`reference/`, `FrozenOrientation.h`, `FrozenDrift.h` and `Frozen.cu` authenticate
both the full original implementation and the narrow extraction. Prior source
receipts retain their old hashes in `reviewed_updates` and compose the checked
drift reversal before recovery/transfer/limiter/group reversals.

The private launch requires positive admitted `row_count` and valid counted
scratch, inherited from recovery scheduling. A null drift tail retains the old
serial path. Startup appends a separate 64-byte aligned row for each attachment;
no casts into another phase's packet, new key, per-step allocation, public
status/diagnostic or source admission is introduced. The recovery key is reused
only after the successful recovery completion kernel on the same stream.

For the qualified V5 dimensions (376930 nodes,11165 attachments,13173 witnesses,
779 groups), the host test measures:

| Quantity | Bytes |
|---|---:|
| New row payload | 714560 |
| Tail alignment | 4 |
| Device growth | 714564 |
| Host metadata growth | 32 |
| `sizeof(CinStorage)` | 664 |
| CIN host forecast | 1375564 |
| CIN device arena | 37831712 |
| CIN arena plus both existing state tails | 62312528 |

These are CIN component forecasts, not a full vehicle allocation claim. The
same exact-limit, one-byte-short rejection and transactional retry checks remain
in the five preceding layout qualifications.

Author gates are six new host functions, thirty preceding host functions, nine
CUDA-shaped C++ syntax units and exact source receipts. These are **not** NVCC,
GPU, native or throughput evidence. Root owns those gates. The new CUDA target
contains three new functions plus the existing actual PART/plain/CIN owner's
late recovered-secondary rotation rejection/discard/retry function. It compares
complete frozen caller packets over three intervals, arbitrary rows across
block boundaries, capture on/off, requested/default-off limiter values,
failing-axis/quaternion prefixes and recovery-before-drift errors.

Root qualification, with the normal serialized workstation guard around each
command (`-j1`, no concurrent owning builds):

```sh
cmake -S lib_utest/qualification/cin_parallel_drift \
  -B /ABS/crash-work/build/cin-drift-root-1 \
  -DCMAKE_BUILD_TYPE=Release -DCIN_PARALLEL_DRIFT_CUDA=ON \
  -DCMAKE_CUDA_ARCHITECTURES=120
cmake --build /ABS/crash-work/build/cin-drift-root-1 -j1
ctest --test-dir /ABS/crash-work/build/cin-drift-root-1 \
  --output-on-failure --output-junit /ABS/crash-work/reports/cin-drift-root-tests-1.xml
```

Owning targets are `//lib_utest/qualification/cin_parallel_drift:host` and
`:cuda`; retain the affected recovery/limiter/transfer/group/screen/ordinary,
complete owner forecast and native regressions. After those pass, root can run
the identical saved V5 benchmark and compare every archive and raw native file.
The baseline event profile measured old `recovery::Complete` at46.334ms/step;
no new speed claim is made before that gate.
