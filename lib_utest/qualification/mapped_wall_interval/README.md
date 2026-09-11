# Mapped wall interval observer qualification

Baseline TL `b85e90485076bad238ba3bf79d6fcfa75c4dcdcf`. This change only
reassociates the same-mask interval observations. Point/parent values, loads,
stiffness, current response, activity, timestep screening, removal potential
and common publication are unchanged. Legacy `MeasureInterval` is the default
and complete fallback, retained verbatim in `reference/` and namespace-adapted
in `SerialInterval.h`. No old tolerance is widened.

`IntervalValues.h` consumes the identical rounded local expressions in their
original order: accepted x-force; force lower/upper/error; stiffness upper;
wall-point y/z; compact addition error; base/prepared x position and velocity;
and actual `kick_dt`. Inactive and zero-force rows are still read. Proposed
activity, current force, other kinematic components and rotations are not new
inputs. Drift uses the actual position difference, not an assumed kick duration.

128-lane trees reduce at most 256 trivial 128-byte summaries. New directed
endpoint sums and upper sums use the existing certified operations. Their new
nominal work gets freshly computed outward radii; it never borrows a serial
radius. Post-loop certificate operations retain original order and run on a
local diagnostics copy. Every failed local/tree operation or insufficient proof
replays the **complete** original routine from original seeds, preserving first
compact-row/global-NID priority and all initialized/partial failure fields.
Only successful tree finalization replaces the diagnostics copy.

## Sufficient original-domain proof

Let `N<=524288`, `u=2^-53`, `eta=denorm_min`, and `M` be the maximum absolute
rounded leaf endpoint, nominal term, upper term and consumed post-loop seed.
Seeds include both potential values/errors, nominal kick/drift offsets, base
resultant endpoints/reaction/moments, and scaled base reaction/moments. No
unconsumed diagnostic field receives a finite check. Exceptional negative error
seeds and negative/nonfinite kick duration use the original routine.

Require `M<=DBL_MAX/[64*(N+1)]`. The signed serial/new prefixes are bounded by
`(N+1)*M` plus directed rounding; relative growth is below 1e-9. The factor 64
also leaves room for two-sum subtraction intermediates and all post-loop
potential/work/radius upper sums. All non-additive local operations are executed
unchanged before certification. Identical base-resultant scaling is still
checked by the original post-loop operations on the local copy.

The non-additive final **moment scaling** needs more than finite prefixes:
`MultiplyScalar` rejects a nonzero product lost to zero. For identical rounded
endpoint leaves, absolute leaf sum `A<=N*M`. Each directed `AddScalar` has error
at most `2*u*(|a|+|b|)+eta`; exact zero additions short-circuit. Serial depth is
at most N and the fixed tree at most N+16. The geometric error series, with
`2*(N+32)*u<1/2`, bounds the old/new endpoint difference by

```
E = 16*(N+32)*u*A + 16*(N+32)*eta
```

`SerialEndpointError` forms A and E outward with existing upper helpers;
it evaluates `A*[16*(N+32)*u]` to avoid a spurious N-squared intermediate.
Any lost positive subnormal product while calculating E causes fallback.
The outward interval `[new_endpoint-E,new_endpoint+E]` contains the old
endpoint. For every channel with any nonzero leaf, it must exclude zero and,
after actual kick scaling, have absolute endpoints strictly above DBL_MIN and
below DBL_MAX/8. This proves old final products neither disappear nor overflow,
and leaves room for final moment radii. Ambiguous cancellation, tiny values or
extreme scales replay the original function. An all-zero leaf channel is exactly
zero in both orders; either sign of zero kick duration has the old zero-scaling
behavior. This proof is a sufficient optimization scope, never new admission.

A nonzero work seed is a carried observer offset, not extra physical work.
The old certificate still bounds unseeded exact physical work. Tests separately
bound the sum of seed plus rounded terms and the corresponding seeded exact
observer (`physical radius + abs(seed)` by triangle inequality). Production
normally supplies zero work seeds; unusual seeds remain part of the supported
fallback contract.

## Ownership and observable route

The separate interval arena tail is at most 32768 bytes on both host staging
and device. Layout gains one 24-byte ArenaRegion; each of two retained Sidecars
gains one 8-byte pointer. Actual `sizeof(Impl)` charges this 40-byte metadata
increase; full original local host forecast rises 32808 bytes and device 32768
bytes on the current 64-bit ABI. Existing complete-source and proof/scratch
reservations remain in force. No per-attempt allocation, extra stream or sync.
Exact/one-byte-short caps and retry are tested. Summary and public mapped
`interval_tree_used` flags fit existing padding; they are still included through
actual sizeof and exact diagnostic identity, never omitted from accounting.

`interval_tree_used` is false at each assembly/candidate start and on serial or
failed finalization; it becomes true only after a successful tree finalization.
It is an execution-route observation, not mechanics evidence. Readback binds
it to the same actual result. Root's full loaded test should record
`last_accepted_step().wall.prepared.interval_tree_used` per accepted interval.
This distinguishes a measured tree route from a numerically equal serial
fallback. No archive schema or app code is changed here.

## Owning tests and root commands

The new target authors five host and three actual CUDA functions. It reuses
`mapped_wall_observers/IntervalTruth.h` in host-only `Truth.cpp` for binary128
physical work, potential defect, impulse and moment bounds. A second check sums
already-rounded work terms with the fixed-depth absolute-term/underflow bound;
serial endpoint reconstruction independently checks E. CUDA packets compare
against the frozen complete routine on faults and against host tree grouping
on successful observers. Actual mixed Q4/T3 point/parent packets exercise
same-mask accepted base, inactive masks and retry with exact physical arrays.
No physical bit differences are accepted.

Controls include 1/127/128/129/263/33001 rows, compact reverse NIDs, half/full
kick, nonzero seeds, zero/negative-zero/negative duration, cancellation at zero
and subnormal nextafter neighbors, scaled overflow, first failure/partial
post-loop fields, unused NaNs and a consumed NaN on a zero-force row. Separate
legacy-default scratch omission and route reset checks are included. Existing
physical_mesh_wall owner test rejects a tampered route identity during its
already-rejected attempt.

Run under root's serialized heavy guard in a fresh cache/report namespace:

```
cmake -S lib_utest/qualification/mapped_wall_interval \
  -B <fresh-build> -DCMAKE_BUILD_TYPE=Release \
  -DMAPPED_WALL_INTERVAL_CUDA=ON -DCMAKE_CUDA_ARCHITECTURES=120
cmake --build <fresh-build> -j2
ctest --test-dir <fresh-build> --output-on-failure
```

CTest targets: `mapped_wall_interval_host`, `mapped_wall_interval_cuda`,
`mapped_wall_interval_identity`. Bazel host target:
`//lib_utest/qualification/mapped_wall_interval:host`; existing owning
`//lib_src/collision:nodal_wall_mapped`. Retain existing mapped_wall_observers,
mapped_wall_evaluation, physical_mesh_wall and legacy contact gates, then the
unchanged full loaded fixture with timing, actual byte growth and route field.

Author work is limited to bounded source/C++-body syntax and configure. No
numerical, native, real CUDA compilation, GPU or original full-source run is
claimed before root qualification.
