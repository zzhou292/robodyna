# Represented interval crossing certificate

`RepresentedIntervalCrossing` owns bounded host staging and immutable published
results for fixed-triangle pairs. `LinearNodalV1` means that each facet vertex
follows the exact real affine path between its two represented binary64
endpoints over normalized time `[0,1]`. `RigidArc` and `Nonlinear` are explicit
unsupported declarations and publish `Unresolved/UnsupportedMotion` without
examining endpoint-union boxes.

The implementation recursively partitions normalized time at exact dyadic
values. At a sampled dyadic time it converts every endpoint to an exact dyadic
integer/exponent value and applies exact triangle SAT, including transverse and
coplanar axes. An intersection is therefore an existence certificate at the
reported exact numerator and depth. A deterministic immutable VF or EE feature
key is selected where one exists; a complete triangle-intersection key remains
for transverse edge/face piercing.

Separation has a different proof. On each accepted time cell, exact quadratic
Bernstein controls certify both triangle normals nonzero. Because every admitted
vertex coordinate is linear, its extrema on that cell occur at the two
endpoints. A strict disjoint coordinate interval therefore certifies separation
for that cell; both child cells must certify before their parent does. This
linear-path endpoint enclosure is never used as evidence of crossing. If exact
contact is not sampled and separation cannot be proved within the declared
depth/work, the result is `Unresolved/WorkExhausted`.

Exact sampled degeneracy is `Unresolved/DegenerateGeometry`; an unsampled
regularity uncertainty also cannot become a separation certificate. The exact
predicate integer has a fixed 16,384-bit stack representation, enough for all
binary64 exponent alignment and predicate products used here. A checked range
failure is explicit `Unresolved/ExactArithmeticRange`.

Pair/path inputs are validated completely, canonicalized by immutable source
path and feature keys, sorted, and duplicate pairs removed before evaluation.
Per-pair work exhaustion is a complete unresolved record. Checked total-work,
result, path, pair, or host-byte exhaustion fails the whole call and preserves
the previous complete publication. A smaller valid query can then retry.

This slice owns no broadphase, active parent-use policy, source selection,
thickness/area/stiffness, force, rigid/CIN response, timestep choice, adaptive
controller, CUDA operation, or physical-owner transaction.

## Author host gate

From a fresh build directory, with one CPU and a 512 MiB guard:

```sh
cmake -S lib_utest/qualification/represented_interval_crossing \
  -B /tmp/represented-interval-crossing -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/represented-interval-crossing --parallel 1
ctest --test-dir /tmp/represented-interval-crossing --output-on-failure
python3 -B \
  lib_utest/qualification/represented_interval_crossing/verify_sources.py
```

Bazel source wiring is
`//lib_utest/qualification/represented_interval_crossing:host_check`; it is a
remaining root-only owning build gate. Root must later compose this API with the
fixed-feature discovery output and run the actual selected initial/short
interval inventory. CUDA/device implementation, full-source counts,
rigid/nonlinear path enclosures, force/STI and common publication remain
separate gates.
