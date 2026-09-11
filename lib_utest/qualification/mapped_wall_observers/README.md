# Mapped wall global observers

This slice replaces only the global `ReduceNodes<true>` fold when the mapped
caller supplies its private observer scratch. The default `parallel::Evaluate`
argument retains the serial fold; the legacy device path is unchanged. The
source receipt binds the complete original evaluation header at 3eca05d, its
exact extracted serial fold, the prior independent whole-evaluation oracle,
and the unchanged physical and interval helpers.

`ObserverValues.h` reuses `nodal_wall_reduction::Sum` and `q4_bounds`. Each tree
merge computes a new force/potential nominal and an outward interval, then
certifies **that nominal**. This is not an old radius attached to a reordered
sum. The seven other sums are reaction XYZ, moment XYZ and surface power.
Maximum penetration retains the original positive-stiffness/strict-maximum
branch. No contact point, share, parent certificate/budget, force, STI, Response,
timestep, scatter or material formula changes.

## Fixed stages and storage

The existing point failure scan and integer parent-rank arbitration run first.
An explicit stream boundary then separates readers from writers. Up to 256
blocks of 128 lanes visit compact-node index `block*128+lane`, with stride
`128*blocks`. Seven fixed halving stages produce each block summary; a second
128-lane kernel loads at most two block summaries per lane and applies the same
fixed tree. There are no floating atomics or per-step allocations.

Each summary is a 144-byte trivial typed record: two named four-double
certificates, seven signed sums, maximum penetration, largest absolute consumed
term and a fallback flag. The maximum arena tail is 36,864 bytes. It is appended
to the existing sidecar, explicitly constructed on the host and copied with the
existing startup arena. HostArena/Layout and both named Sidecar pointers are
accounted through existing `sizeof(Impl)` and sidecar-byte forecasts. The
old mapped arena cap now includes this tail; no cap increases. `ObserverBlocks`
checks 1..524,288 before extent arithmetic. The host test verifies exact cap,
failed-output preservation, alignment/binding and maximum extent. This is
in-process layout, not a persistent archive compatibility change.

## Serial-domain preservation

The fast-path predicate proves the old serial observer arithmetic would finish:

- All consumed force/potential nominals and endpoints (including the identity
  seed) are finite and nonnegative; endpoint order is valid. Input `error` is
  intentionally ignored because the original `Sum` recomputes its radius.
- With `N = nodes+1 <= 524289`, the largest absolute consumed scalar is at most
  `DBL_MAX/(32*N)`. This is a sufficient proof, never an admission rejection.
- Every exact prefix magnitude is at most `DBL_MAX/32`. For binary64 RN with
  gradual underflow, at most N directed additions grow a positive endpoint by
  less than 1e-9 relative (use `(1+2u)^N`, `u=2^-53`); the accumulated absolute
  subnormal allowance is below `2*N*denorm_min`. Thus all prefixes stay far
  below `DBL_MAX/16`. Signed nominal sums obey the same absolute-term bound.
- The original two-sum subtraction intermediates, certificate differences
  between nominal/endpoints, and their outward rounding remain finite within
  this margin. Nonnegative input endpoints also preserve the original serial
  interval/certificate domain. A malformed interval cannot be hidden by the
  new grouping. Any local/tree arithmetic failure selects fallback as well.

Failure of any predicate, exceptional penetration, or prior point/parent error
runs the complete frozen serial fold against the **untouched original** global
diagnostics. This retains first compact-source node failure (not minimum NID),
partial force-before-potential updates, partial signed nonfinite diagnostics,
and final count/rate publication. A rejected point-admission stage still exits
without those publications. No accepted history is modified by either path.

## Numerical truth and derived certificates

The independent C++ binary128 oracle sums the already-rounded leaf values and
leaf interval endpoints. For signed/nominal sums it uses
`2*(gamma_d*sum(abs(term)) + A*denorm_min/(1-d*u))`, equivalently the expression
in `Truth.h`, where `gamma_d=d*u/(1-d*u)`,
`d=ceil(nodes/(128*blocks))+7+ceil(blocks/128)+7+1`, and
`A=nodes+256*blocks+257`. This covers fixed path depth, cancellation and every
possible gradual-underflow addition; factor two also covers the much smaller
binary128 accumulation error. Packet-specific corruption controls must exceed
that bound. No global comparison tolerance is widened.

Force/potential lower and upper fields must enclose the independently summed
input endpoints; the new radius must cover the new nominal against both.
Dyadic, signed cancellation, subnormal/zero and near-overflow controls are
included. Extreme cases test both old successful prefixes and old failures.

`MeasureInterval` is unchanged. Its observer-dependent base potential/error,
potential increment, conservative defect/work uncertainty and impulse/moment
certificates may legitimately differ. Host and CUDA tests compare these to
independent high-precision point-packet work/impulse/moment truth and retain
bit comparisons for its unchanged kick/drift/quadratic loops. This slice does
not parallelize that serial loop or `RemovedPotential`; it does not claim a
new large-step or extreme-range admission theorem for those separate routines.

## Gates

Author evidence (one CPU, 512 MiB address-space limit): five host functions
pass, including 33,001-node/max-summary traversal and exact-cap checks. Owning
CMake host and source-identity tests pass. C++ syntax checks cover production,
new CUDA-shaped tests, the default old evaluator and actual mapped owner test.
This is not CUDA or native numerical execution.

Four CUDA functions are authored for root execution: complete mixed Q/T parent
masks and frozen/default physical packets; first point/parent/global failure
and retry; accepted-base copy/candidate retry and derived certificates; all
256 summaries with grid-stride, cancellation and subnormal controls. Successful
packet comparisons use named fields, not compiler padding. No numerical result
from these CUDA tests is claimed before the root gate.

Under the existing workstation guard, root can run:

```sh
cmake -S lib_utest/qualification/mapped_wall_observers -B "$BUILD" -DMAPPED_WALL_OBSERVER_CUDA=ON -DCMAKE_BUILD_TYPE=Release
cmake --build "$BUILD" --parallel 2
ctest --test-dir "$BUILD" --output-on-failure
```

Affected owning regressions are `mapped_wall_evaluation` (default omitted
scratch), `physical_mesh_wall` with `PHYSICAL_MESH_WALL_CUDA=ON` (both production
opt-ins), and `//lib_src/collision:nodal_wall_mapped` plus
`//lib_utest/qualification/mapped_wall_observers:host`. Then root repeats the
complete loaded accepted/discard/retry gate with timing and device growth.
Only that run establishes the actual performance benefit and whole-case
resource margin; the six-GiB device-growth guard remains unchanged.
