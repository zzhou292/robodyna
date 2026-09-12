# Mapped QBAT ordered gather qualification

Baseline: `01a9a393dab74b35535ec9a36b8adb875402dc99`. This is a scheduling
change. The frozen complete assembly, candidate kernel and measurement bodies
are retained with reversible include/namespace wrappers. The source receipt
checks those original bytes plus the unchanged force, result, node, stiffness
and scalar assembly leaves. No numerical tolerance is introduced.

The four host functions cover exact eight-channel source-order sums with
nonzero incoming force/couple/stiffness, cancellation and signed zero; all
active/removed masks; late incidence rejection before writes; inclusive arena
caps; and signed diagnostic/error-prefix agreement. The four CUDA functions
compare the frozen full kernels, including virgin and carried packets, competing
validation/addition failures, late failure, complete destination rollback, fresh
input retry, complete candidate readback fields and accepted-cache preservation.
The candidate cache uses the existing qualified force path, not a new oracle.
Native mechanics and real owner/token behavior remain the existing owning gates.

Author evidence (one CPU, 512 MiB; no NVCC/native/GPU execution):

- `qbat-mapped-gather-author-final-3.json/xml`: four host functions and five
  source verifier entry points pass.
- `qbat-mapped-gather-author-host-regression-2.json`: all four QBAT and ten
  affected QEPH/T3 host functions pass. Its final command used a nonexistent
  QBAT verifier path; that orchestration error is preserved and corrected by
  final-3. `qeph-gather-qbat-author-2.xml` and `t3-gather-qbat-author-2.xml`
  retain the affected host results.
- `qbat-mapped-gather-author-syntax-2.json`: 14 C++/CUDA-shaped body checks,
  including both unchanged Q/T family adapters. These do not compile CUDA.

The original V5 count-only layout (4250 QBAT parents, 376930 owner nodes,
zero curve points for this isolated layout control) is 82,042,528 device bytes.
Its optional mapped tail is 29,024,792 bytes beyond the same new-header legacy
layout. Complete startup forecasts also charge that tail in their host staging;
actual retained curve points and other participants remain separately accounted.
No allocation occurs per attempt. Ordinary QBAT modes allocate no optional tail.

Root's new gate, under the existing normal guarded heavy lane:

```sh
cmake -S "$TL/lib_utest/qualification/qbat_mapped_gather" -B "$BUILD" \
  -DCMAKE_BUILD_TYPE=Release -DQBAT_GATHER_CUDA=ON -DCMAKE_CUDA_ARCHITECTURES=120
cmake --build "$BUILD" -j4
ctest --test-dir "$BUILD" --output-on-failure
```

Required affected root gates:

- Existing `qbat_mapped` with `QBAT_MAPPED_NATIVE=ON`,
  `QBAT_MAPPED_CUDA=ON`, `QBAT_MAPPED_ORIGINAL=ON`; it includes resident host
  regression and the original 4250-parent source model.
- Existing `qbat_resident` actual native/owner recurrence and shared physical
  publication; these cover acceptance, stale/missing identities and claims.
- `qeph_mapped_gather` / `t3_mapped_gather` with their respective
  `QEPH_GATHER_CUDA=ON` / `T3_GATHER_CUDA=ON`, including source identity.
- Owning Bazel targets `//lib_src/elements/qbat:batch`,
  `//lib_src/elements/qeph:batch`, `//lib_src/elements/t3:batch`,
  `//lib_utest/qualification/qbat_mapped_gather:host` and `:cuda`.
- The same complete V5 loaded prefix and synchronized stage timers, to measure
  actual impact. There is no speedup claim before that run.

No QEPH/T3 force or constitutive code, activity readback, timestep admission,
rigid response, publication coordinator or application profile is changed.
