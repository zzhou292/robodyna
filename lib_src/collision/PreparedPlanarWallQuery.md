# Prepared finite planar-wall query filter

This is a numerical FP64 filter for the existing `planar_detail::FindOwner`,
not an exact geometric predicate or a different contact model. It copies the
complete ordered `WallFace` array, prepares padded Y/Z boxes once per upload,
and invokes the unchanged `ClosestPointOnTriangle` on every unpruned face.
The original distance comparison and smallest stable face-ID selection remain
unchanged, including first-in-array behavior for equal IDs. Equality with a box
boundary is never pruned. It does not replace finite-wall topology, hole,
clearance or whole-sweep validation.

`Initialize` admits a bounded nonempty copied packet, including geometry that
is ineligible for acceleration. Invalid pointer/count arguments preserve an
existing packet. The fast path requires every face to satisfy all eligibility
checks. One invalid or uncertain late face disables filtering for the entire
packet. Each query additionally requires finite coordinates, exactly the saved
common X and the safe magnitude bound; otherwise the complete original scan
runs. No invalid distant face can disappear behind a box. Query output follows
the original partial-publication contract: reset owner, keep point if no match,
and preserve an earlier selected owner/point when a later face fails. The
optional diagnostics identify filter use and the number of skipped faces;
they do not claim an observed count of calls inside the fallback function.

The prepared object is immutable during queries and contains its own faces,
so an external array cannot become mismatched with saved bounds. It is a
trivially copyable execution-space packet, not authentication of arbitrary
mutated bytes. Its fixed 512-face capacity is the existing wall capacity;
copied geometry and boxes occupy less than 80 KiB. The source qualification
prepares directly from each uploaded packet, including deliberate mutations,
and includes preparation plus the extra transfer in checked end-to-end time.

The eligibility proof is separate from the padding proof. Require all absolute
vertex coordinates and query Y/Z coordinates at most `2^100`, every nonzero
edge maximum component at least `2^-100`, and a common exact X. Edge and query
differences are then bounded by `2^101`. The normalized segment direction has
maximum component one, while its length factor is at least approximately
`2^-201`, far inside normal binary64. Segment division/projection and blends
cannot overflow, and no positive length factor can underflow to zero.

For the triangle branch, normalize the two edges by their shared maximum
component. Their components have magnitude at most one; the largest squared
edge is at most eight up to the ordinary arithmetic rounding. The computed
planar determinant must have magnitude at least `4096*DBL_EPSILON`, an eightfold
margin over the old `64*DBL_EPSILON*8` degeneracy bound. Because the cross has
only its X component, `hypot(hypot(x,0),0)` is exactly `abs(x)`. Normalized query
components are bounded by `2^201`; cross products and division by the admitted
determinant remain below `2^250`, well below overflow. Accepted weights lie in
`[0,1]`; weighted points and distances also remain finite. A startup call to
the original operation provides an additional eligibility check. Faces outside
these sufficient conditions are not rejected: they retain the original scan.

Let `e=DBL_EPSILON`, and `M=max(1, |vertex_yz|, tolerance)`. The returned segment
point is an endpoint or a rounded blend with `t` in `[0,1]`. The face point uses
nonnegative weights `b,c`, `s=RN(b+c)<=1`, and `a=RN(1-s)`. Their real sum differs
from one by at most `2e`. For either construction, bound the real point outside
the vertex interval by `2e*M`, three rounded products by `3e*M`, and the two
rounded additions by `4e*M`; allowing higher-order terms gives `12e*M`.
Subnormal absolute errors are covered by the deliberately conservative `M>=1`.
The bound concerns the actual computed convex combination, so ill-conditioned
barycentric estimates cannot move an accepted point outside it unnoticed.

Conditional on the original computed distance being at most the tolerance,
coordinate subtraction and the two nested hypot evaluations require at most
an additional `16e*M` under the pinned strict runtime accuracy assumptions.
Thus `64e*M` exceeds the sum of point-construction and distance allowances;
it is derived conservatively and is not tuned against observed source error.
The saved lower/upper boxes add the original tolerance and that padding with
the existing directed arithmetic. A represented query strictly outside these
closed boxes cannot be a tolerance match under those numerical assumptions.

The hypot assumption is intentionally explicit. NVIDIA publishes small ULP
accuracy bounds (2 ULP for double hypot) but describes such bounds as observed,
not universal formal guarantees; the host operation also uses its pinned libm. This filter makes no
all-libm theorem. See the primary [CUDA floating-point documentation](https://docs.nvidia.com/cuda/archive/13.1.0/cuda-programming-guide/05-appendices/mathematical-functions.html).
Strict FP64, no FMA contraction, no fast-math and no FTZ remain required. Host
adversarial equivalence and the actual source CUDA equivalence/cost gates
qualify the concrete build/runtime, with the measured scope below.

Six host functions in `lib_utest/utest_prepared_planar_wall_query.cc` compare
every returned point/feature/normal/distance/weight and owner to the unchanged
scan. They cover interiors, stable seams, vertices, outside/hole cases, face
permutations, exact tolerance boundaries and neighboring doubles, translations
and scales, skinny/off-plane/extreme cases, late invalid/overflow fallback,
immutable copies and failed preparation/retry. The source 100/100/400-face CUDA
gates retain their original physical budgets, 128 workers, <=2 MiB combined
explicit storage, and original 2 ms/5 ms/20x performance conditions. No new
law, timestep, mass policy, owner or production selector is introduced.
The existing source CUDA rollback function additionally requires full fallback
for a last NaN, degenerate or overflow face, preserves the prior full result,
and retries cleanly after each. A valid one-ULP off-plane query checks successful
fallback against the old host scan. Reports identify the acceleration backend
separately from the unchanged physical nodal-wall model.

The first prepared-query execution passed all six owning host functions:
[configure](../../../crash-work/reports/prepared-wall-query-configure-1.json),
[build](../../../crash-work/reports/prepared-wall-query-build-1.json),
[guard](../../../crash-work/reports/prepared-wall-query-tests-1.json), and
[XML](../../../crash-work/reports/prepared-wall-query-xml-1/utest_prepared_planar_wall_query.xml).
The actual source suite also passed all six functions, including its original
cost conditions: [configure](../../../crash-work/reports/prepared-wall-source-configure-1.json),
[build](../../../crash-work/reports/prepared-wall-source-build-1.json),
[guard](../../../crash-work/reports/prepared-wall-source-tests-1.json), and
[XML](../../../crash-work/reports/prepared-wall-source-xml-1/robo_dyna_source_part_nodal_wall_check.xml).

For the fixed 117-node/94-parent original100-face coherent profile, one warm
plus five checked calls measured candidate median 0.984992 ms, maximum
0.991136 ms and checked end-to-end median 1.078937 ms. The contemporary unchanged
integral median was 428.126343 ms, a 434.649551x ratio for the explicitly
different recorded kernel scopes. Candidate storage is 459,376 bytes; the
combined candidate/integral allocation is 1,981,840 bytes, below the unchanged
2 MiB cap. Original/flip100/subdivide400 numerical checks and late fallback/
retry checks passed. The source nodal-versus-integral force and potential model
differences remained exactly unchanged; this filter changes no contact law.

The prior 3.405312 ms median failure remains in the
[first-execution snapshot](../../../crash-work/checkpoints/source-nodal-wall-first-execution-1/manifest.json).
Its measured query-cost diagnosis remains in the
[profiler snapshot](../../../crash-work/checkpoints/source-nodal-wall-profile-1/manifest.json).
This is a passed prescribed source-contact query/cost gate under the documented
numerical assumptions. It does not establish connected dynamics, original
source mass/formulation equivalence, whole-vehicle throughput or a Yaris crash.
