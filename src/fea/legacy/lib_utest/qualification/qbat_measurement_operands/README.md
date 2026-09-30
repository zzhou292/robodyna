# Mapped QBAT measurement operands

Baseline `b1415d3f425eefb99ff93b2a254de1f9e88abcb4`. The measured mapped
`FinalizeMapped` kernel took 44.525 ms/call in the supplied diagnostic profile.
This slice moves independent measurement operands into the existing parallel
parent-validation launch. It does not change native mechanics or reduce sums
in a different order. No speedup is claimed before the owning full-prefix run.

`MeasurementParent` contains 20 doubles and three byte flags, padded to 168 B.
The eight native ledger operands, four individual kick operands and four drift
operands remain distinct until the original source/channel/slot fold. Four
extrema inputs complete the numerical packet. `AccumulateInternalWork` is
unchanged; its existing template writes the operands of each `-=` expression,
without applying a negation or accumulating a local partial sum. Every admitted
material, parent count and active/removed state uses the same path. There is no
Yaris, no-deformation, zero-force, or zero-couple shortcut.

The producer first checks `candidate_status`, then the complete unchanged
`ValidResult`. Failed rows overwrite their packet and never inspect stale trial
history. The finalizer retains the complete element-status scan before any
measurement error, exact partial diagnostics at the first invalid result, and
the delayed aggregate finite checks. Derived work overflow does not become an
early leaf rejection. The displacement kernel, its invalid-node serial fallback,
and its summary/finite suffix remain unchanged. Trial/accepted result slabs,
control identity, public full readback and common commit/discard are unchanged.

There are still three mapped measurement launches on the borrowed owner stream,
one control readback, and one resident allocation. The packet has no independent
time/attempt cache and is completely overwritten for every candidate. Unmapped
execution retains the original serial caller and has no operand rows.

The 64-bit ABI gate confirms Storage 656 -> 664 B, Layout 456 -> 480 B,
AssemblyMemory 48 -> 56 B, and private Impl 181376 -> 181408 B. The new mapped
device extent is `168 * parent_count + 8` bytes above baseline. Complete
standalone QBAT startup forecast growth is `168 * parent_count + 40`, including
the temporary startup mirror and retained private descriptors. At 4250 parents
these are 714008 device bytes and 714040 standalone host admission bytes.
`ShellMappedFootprint::MakeFootprint` partitions the latter into 714008 temporary
bytes and 32 retained participant bytes. The app initializes participants
sequentially and takes the maximum temporary phase in
`case/vehicle_runtime/Forecast.cpp` and `case/vehicle_wall/RuntimeBudget.cpp`.
Consequently the qualified V5 runtime and complete-run host bounds grow by only
32 bytes; the QBAT temporary phase does not control either maximum. This is
phase-based allocation accounting, not a measured RSS reduction or shared
simultaneous storage. `InitializeMapped` retires its local HostArena after the
synchronous startup upload before the next participant initializes.

A bounded count-only query of the actual layout at 4250 parents, 376930 nodes
and the maximum 1024 curve points bounds the new mirror by 82772920 bytes.
Adding its 45410248-byte owner proof gives at most 128183168 temporary bytes;
the two required QEPH result slabs alone occupy 923019712 bytes. Thus the
non-controlling QBAT phase has at least 794836544 bytes of margin below an
already required phase, independently of its actual curve count. Evidence:
`crash-work/reports/qbat-forecast-margin-author-1.{json,log}` and the root V5
`qbat-operands-vehicle-functions-1` reports. No production accounting change.

Unmapped growth is only 8 device/40 standalone host forecast bytes;
the optional packet count is zero. Existing alignment/overflow/inclusive-cap
checks account for every allocation. No per-step allocation occurs.

Author checks use one CPU and 512 MiB, with no NVCC/native/GPU execution:

- Eight host functions cover full frozen-caller diagnostic bits, all parent
  removal masks, partial surface masks, linear/table hardening, rate histories,
  prescribed/coupled usage, nonzero work, signed zeros/cancellation, invalid raw
  bools, derived overflow, competing failures, stale packet avoidance, retry,
  unequal node/parent counts and exact optional arena caps.
- Three CUDA-shaped C++ units check the real production launch unit and the two
  owning CUDA test units. This is not a CUDA compilation claim.
- The source verifier preserves the full earlier serial caller and all prior
  gather receipts. It checks the exact fold adaptation, complete result
  validation, original launch count/order, and displacement/finite suffix.

The authored CUDA gate has three frozen full-caller functions (including
multi-block counts, complete result fields and failure/retry) and two actual
owner functions (output/accepted-history preservation and exact forecast caps).
The existing gather/reference tests remain separate regression obligations.

Root commands, under the existing heavy guard:

```sh
cmake -S "$TL/lib_utest/qualification/qbat_measurement_operands" -B "$BUILD" \
  -DCMAKE_BUILD_TYPE=Release -DQBAT_MEASUREMENT_CUDA=ON \
  -DCMAKE_CUDA_COMPILER=/usr/local/cuda/bin/nvcc -DCMAKE_CUDA_ARCHITECTURES=120
cmake --build "$BUILD" --parallel 1
ctest --test-dir "$BUILD" --output-on-failure -j1
python3 -B "$TL/lib_utest/qualification/qbat_measurement_operands/verify_sources.py"
```

Reuse the existing root caches for `qbat_mapped_gather` (`QBAT_GATHER_CUDA=ON`),
`qbat_mapped` (`QBAT_MAPPED_NATIVE/CUDA/ORIGINAL=ON`), and `qbat_resident`
(`QBAT_RESIDENT_NATIVE/CUDA=ON`). The last includes native point/force history,
public full readback alias/copy-failure tests and real common publication. Keep
the local gfortran 11.4 selection used by those native caches. No donor changes
require regenerating native fixtures. Owning Bazel targets are
`//lib_src/elements/qbat:batch`, `//lib_utest/qualification/qbat_measurement_operands:host`
and `:cuda`, plus the existing gather/owner targets. The new real-owner gate is
CMake-owned and reuses `qbat_mapped/OwnerFixture.cu` directly.

Finally repeat the same V5 prefix and compare all non-timing archive outputs
exactly, with normal resource guards. The status scan and ordered scalar fold
remain serial; this increment does not claim to eliminate their cost.
