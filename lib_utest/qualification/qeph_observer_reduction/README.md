# Mapped QEPH observer reduction

This changes only the global diagnostic reduction for mapped, joined QEPH.
Root's durable controller step reported3.538s in QEPH candidate work;
that stage includes material kernels, diagnostics and synchronization. This
increment removes the remaining successful-path serial parent/node scans.
It is not yet a measured speedup or a whole20ms-run performance claim.
The later root loaded gate atTL78206bc, still before this increment, reports
3.294s per QEPH candidate and62.17s total startup/three-attempt runtime.

The eight reassociated binary64 sums are `internal_work[0/1]`,
`internal_work_increment[0/1]`, `hourglass_viscous_work`, its increment,
`internal_kick_work` and `internal_drift_work`. The six exact extrema are minimum
area/thickness/native dt and maximum displacement/absolute strain/thickness
curvature. Global kinetic channels remain unavailable for the joined profile.
No diagnostic sum feeds element history, forces, nodal assembly or timestep.
Live accepted/prepared receipt comparison remains exact.

## Fixed stages and fallback

1. Unchanged `CandidateElements` advances each material/history independently.
2. Existing disjoint preparation writes mapped result validity and each node's
   displacement/quaternion scratch. It never reads an unwritten failed result.
3. `ObserverKernels.cu` uses128 threads per block and
   `min(256,ceil(max(parents,nodes)/128))` active blocks. Each lane folds its
   fixed source-index subsequence; each block performs a fixed halving tree.
4. One128-thread block reads those summaries in fixed lane order and performs
   the same tree. Its lane0 publishes one private control, or calls the old
   serial finalizer. All dependencies use the existing owner stream.

No floating-point atomics, parent-sized partial arrays, second owner/clock or
per-step allocation are introduced. `ShellBatchFields::AccumulateInternalWork`
keeps its original expression body and its default `double` accumulation.
The observer's tiny `WorkObservation` sink captures each of the four already
rounded slot terms before adding it to a channel. There is no per-parent
substitution for those terms. Existing FP options disable reassociation/FMA
contraction and flush-to-zero; per-element arithmetic is unchanged.

Any individual element/result/node error, missing roles, nonjoined/kinetic
scope, nonpositive native-dt tie, intermediate sum overflow or inadequate
finite-prefix proof invokes the complete old `FinalizeDiagnostics`. Thus
element-update failure still precedes missing catalog, result validation,
node checks and parent observations; first indices and private partial failed
diagnostics are preserved. Rejected public output and accepted caches remain
unchanged. Nonpositive native dt is not newly rejected by the optimization.

Let `N=4*parents+1` bound every channel's number of rounded leaf terms including
its initial seed, and let M be their maximum absolute value. The fast path
requires finite inputs and `M <= DBL_MAX/(8*N)`. The admitted count is at most
UINT32_MAX. Hence every original serial prefix and every new tree partial has
an absolute term sum below DBL_MAX/8; binary64 roundoff cannot consume the
factor-eight margin. This intentionally conservative test handles rounded
division as well. If it fails, the serial path decides admission, including
cases whose true final cancellation is finite but an original prefix overflows.

## Observer rounding boundary

For the exact sum S of already-rounded leaf terms, let A be their absolute sum,
u=2^-53, B the active block count and R=ceil(parents/(128*B)). The maximum
addition depth is conservatively

```
d = 4*R + 7 + ceil(B/128) + 7 + 1
```

The final1 covers the original identity seed. A bound is
`gamma_d*A + E_underflow`, with `gamma_d=d*u/(1-d*u)`. The qualifier uses
`2*(d*u*A + K*denorm_min)/(1-d*u)`, where
`K=4*parents+2*B*128+2*128+1` bounds all potentially rounding additions for a
channel. The explicit absolute sum handles cancellation; the additive term
covers gradual underflow. The factor2 also covers the much smaller binary128
oracle accumulation error for these bounded packets. Comparison to mathematical
unrounded leaf expressions would additionally require their operation error;
this gate deliberately compares the existing rounded leaves.

These are ordinary work observations, not wall certified bounds. No public
interval certificate, diagnostic tolerance or generic `NearlyEqual` changes.
The independent host oracle recomputes rounded cache terms without calling the
production accumulator, sums them in binary128, and verifies a representable
corruption outside each packet's bound is rejected. CUDA compares the actual
new kernels against the frozen serial device routine; only these eight sums
may differ. Extrema, source/owner/phase identity and untouched packet bytes
remain exact. Repeated trees must produce identical diagnostics.

## Storage, compatibility and gates

`ObserverSummary` is a trivial128B record in the existing mapped arena tail.
At most256 records add32,768B; one named pointer adds8B to the private header.
The full324,094-parent/372,435-node base arena is1,183,523,832B, excluding its
unchanged material sidecars. `MakeForecast` already charges this arena's host
staging and device allocation. Legacy initialization has no observer tail;
its header-size effect is accounted. These are in-process layouts, not archives.
Host/device mapping uses named typed regions, with no alias/lifetime overlay.

Six host functions cover1/127/128/129/263/33,001-parent trees, signed cancellation,
independent high-precision bounds, all masks/epochs/assembly and skin defaults,
signed-zero/NaN untouched seeds, competing late failures and retry, successful
extreme cancellation versus original-prefix overflow, subnormals, count guards
and full-count exact/one-byte-short caps. Eight affected original diagnostics
and deterministic assembly host functions also pass. Three CUDA functions are
authored for actual kernels, unchanged mechanics/cache bytes, independent host
truth, repeatability, competing failures, extreme fallback and successful retry.

Author jobs use1CPU/512MiB; no author NVCC, native or GPU execution. Root owns:

```
cmake -S lib_utest/qualification/qeph_observer_reduction -B BUILD \
  -DCMAKE_BUILD_TYPE=Release -DQEPH_OBSERVER_CUDA=ON
cmake --build BUILD -j2
ctest --test-dir BUILD --output-on-failure
bazel test //lib_utest/qualification/qeph_observer_reduction:host
bazel build //lib_src/elements/qeph:batch
```

Rerun original mapped Q/T native/CUDA and gather/validation qualifiers. The old
direct `LaunchMappedCandidateDiagnostics` remains available to that validation
qualifier and the unchanged serial finalizer remains the production fallback;
normal mapped Batch evaluation dispatches `LaunchMappedObserverDiagnostics`.
Then compare complete loaded accepted/retried intervals: all histories, nodal
forces, owner state, masks and selected timestep must match; only the eight
observations use their explicit sum bounds. Recheck full forecast/device growth
under the existing6GiB guard and measure stage timings. No new native equation
or constitutive qualification is claimed by this observer-only change.
