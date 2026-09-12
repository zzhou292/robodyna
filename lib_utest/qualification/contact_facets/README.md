# Fixed physical contact facets

`FixedContactFacetBinding` retains an admitted centered-plane S0 binding and
one shared Q4/T3 subdivision template. `WeightedReferenceSurfaceV1` freezes
level 0, 1 or 2 (`n=1,2,4`). It is independent of render triangulation. Virtual
vertices introduce no DOFs, mass, history, clock, or contact admission.

Each Q4 has `2*n*n` triangles; each T3 has `n*n`. `Describe` resolves one
parent/local-facet value, preserving source instance, family/index, EID/PID,
original cyclic domain nodes, law, material-point applicability and unchanged
reference half-thickness. Rigid skins remain zero-point geometry. Shared edge
keys use ascending original NIDs and a reduced dyadic fraction, so reversed
Q4/T3 winding conforms. Interior seams retain parent identity and remain
distinct from physical parent boundaries. Coincident different NIDs never weld.
These keys do not decide unique force, area, thickness or active-use ownership.

`ComposeFacetPoint` computes original-node weights in source-slot order from
triangle barycentric weights. Call the existing `weighted_surface/Mapping.h`
helpers for position, velocity and transpose force projection. The composed
weights must not be replaced by Q4 shape functions at averaged natural
coordinates: those differ on a warped Q4. These pure value helpers authenticate
neither source identity nor an owner stage; pointee extents and nonoverlap are
caller contracts. The retained binding protects its complete source and arena
from output aliases and publishes only after complete validation.

## Current approximation enclosure

On `[0,1]^2`, write the native Q4 as `P=a+b*u+c*v+D*u*v`, with
`D=p0-p1+p2-p3` in native cyclic order. On either triangle of a square cell of
width `1/n`, the scalar interpolation error of `u*v` is at most `1/(4*n*n)`.
Thus the pointwise error and corresponding two-sided Hausdorff upper bound are
`norm(D)/(4*n*n)`. T3 has zero polynomial approximation error. This bound can
overestimate geometric distance for planar non-parallelograms.

`Approximation` reads only the parent's current position slots. Existing
certified signed intervals enclose D before an outward norm/product. Existing
source-order weighted interval sums enclose both exact and represented virtual
vertices; their full interval widths give an additional outward vertex error.
The sum bounds the native polynomial against linear facets of those represented
vertices. It does not enclose a subsequent closest-point implementation or all
rounding in a composed witness. Overflow or unrepresentable positive terms
reject without changing output. Degenerate geometry can have zero error: this
query does not establish current regularity or collision admissibility.

The error is quality/search metadata, **not an added physical radius or gap**.
Making it configuration-dependent contact thickness would require its missing
derivative. No exact bilinear closest point, continuous collision detection,
normal/curvature bound, initial-intersection handling, contact force or runtime
transaction is claimed. Subsequent VF/EE work must also handle already
intersecting triangle pairs; separated-feature distances alone are insufficient.

## Memory and tests

Only the templates are allocated, at most 40 vertices and 48 triangles across
both families. No per-parent facet soup is owned. Forecasts count the full
retained S0 payload, handle/implementation and shared-control reserve, exact
template arena, and fixed typed startup/query staging. Caller output arrays,
allocator bookkeeping and process RSS remain outside this payload convention.
Default limits admit 2,048 parents/65,536 facets/64 MiB; explicit Vehicle limits
admit 524,288 parents/16,777,216 facets/2 GiB. These do not alter workstation caps.

Eleven host functions cover all three shell families and rigid-skin roles;
level/count/byte caps and retry; retained lifetime and output aliases; reversed
Q4/T3 edges and distinct coincident layers; warped composed mapping, force,
moment and virtual work; affine/current warped enclosures at every level;
independent exact dyadic probes, hidden signed cancellation, three-component
norms, represented vertex rounding, and late invalid/overflow/subnormal bounds.
The exact dyadic qualifier requires a host long-double significand of at least
64 bits; production has no long-double dependency.

Root owning gate, from a source checkout:

```sh
cmake -S lib_utest/qualification/contact_facets -B /tmp/contact-facets-root \
  -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/contact-facets-root --parallel 1
ctest --test-dir /tmp/contact-facets-root --output-on-failure
```

Bazel: `//lib_utest/qualification/contact_facets:host_check`. Affected source
gate: `//lib_utest/qualification/self_contact_surface:host_check`; shared algebra:
`//lib_utest/qualification/weighted_surface:host_check`. No native/CUDA source is
compiled by this CMake gate. The existing physical-source chain locates CUDA
headers for declarations only. Fixture exports are additive; no historical
receipt or frozen arithmetic is changed.

Author evidence: `crash-work/reports/contact-facets-author-3.json` and its
`commands.json`/`functions.xml`: all five new production translation units and
four test units compiled, 11 functions passed under 1 CPU/512 MiB. The test link
reused unchanged read-only S0 host libraries. Root must still run the owning
build. Prior failed evidence is retained: missing test `<climits>` (attempt 1),
then an overstrict zero-roundoff assertion (attempt 2); no numerical rule was
relaxed to resolve either.

## Original source gate still pending

Reuse the actual prepared V5 S0 subset, including its unsupported-plane receipt.
For each level log every forecast, iterate all retained parents and compare
source/role/thickness and boundary keys against S0. Stream descriptors instead
of allocating a facet soup. Query actual stamped current positions one parent
at a time, report quality-bound extrema independently of thickness, and retain
all failed geometry/budget evidence. No actual V5 warp census was run here.
Uniform all-shell upper inventory is 328,344 Q4 plus 21,301 T3, hence
677,989 / 2,711,956 / 10,847,824 facets; the admitted S0 subset determines the
real counts. Adaptive refinement, pair ownership, CCD and owner integration
remain separate increments.
