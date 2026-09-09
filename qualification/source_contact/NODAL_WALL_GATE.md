# Prescribed source nodal wall gate

This app-only qualification composes TL's separately named
`reference-area-lumped-nodal-wall-v1` model with the authenticated PID2000157
fixture. The 88 original Q4 and six native T3 references remain unchanged;
their 370 area shares assemble into 117 real physical-node indices. Contact
weights do not define mass: the retained E2a16 equal-native-node fixture mass
is reused and `source_mass_equivalence=false` remains explicit.

`SourceNodalWallFixture` initializes fresh immutable Q4/T3 references and copies
their typed parent weights. `BuildInput` validates every parent's two endpoints,
real mass/fixed masks and entire projected straight sweep through the existing
interval and `PlanarWallBox` operations. Both fixed endpoints must be unchanged,
have zero velocity and certified nonpositive penetration. All parents must lie
inside the actual finite wall with the existing clearance; holes, uncovered
edges and unresolved coverage reject the whole packet. No current shape or
reference area is flattened. The wrapper owns no mechanics state or clock.

`SourceNodalWallCuda` copies at most 400 validated wall faces. Each upload
prepares the owning TL [finite-wall query filter](../../../Total-Lagrangian-FEA/lib_src/collision/PreparedPlanarWallQuery.md)
from those exact faces. Conservative boxes prune only safely excluded faces;
survivors use the original closest-triangle calculation and stable owner rule.
Ineligible geometry or queries use the complete original `FindOwner` scan.
One block of 128 threads performs this query, evaluates existing raw point laws
for each physical node's parent shares, checks each parent budget, then reduces
the unique nodes in fixed order. Shared force is added once; parent diagnostics
are not another spring. Per-parent N/J budgets stay 5e-7 and
1.2500000000000005e-12; global uncertainty is the outward sum. A separate trial
and published result preserves output on late semantic failures. CUDA errors
poison the qualification object; safe invalid-launch injection tests this path.
Device endpoint lookup is additional checking and does not replace the host
sweep proof or authenticate modified raw packet bytes.

Six GTest functions cover three test translation units:

- All 94 separate rigid-shift parent experiments compare nodal resultant and
  potential with the unchanged material-integral model using both measured
  certificates. Jensen inequalities are checked within these radii; model
  discrepancy is reported independently of arithmetic error.
- Actual source sweep and fixed/endpoint-cap failures preserve the whole host
  packet. A removed triangle is an explicit negative hole fixture; the same
  shifted source sweep first has to pass the intact canonical wall.
- Actual GPU/owning-host parity retains every source identity, parent and node
  certificate, force, reaction moment, power, mass-scaled row and query owner
  across original100, flip100 and subdivide400 meshes.
- A positive source node on an internal seam selects the smallest stable face
  ID and one spring. A missing finite face rejects on device as well.
- Late mass, outside, adjacency and parent-budget failures preserve the prior
  result, followed by exact fieldwise clean retry; fixed nodes and CUDA poison
  are separate paths. No tests skip a missing or failed GPU runtime.
- For the ORIGINAL100-face coherent profile, one warm plus five checked calls
  per implementation retain the frozen engineering thresholds: nodal event
  median <=2 ms, maximum <=5 ms, and median speedup >=20x against the unchanged
  contemporary integral baseline. The nodal event includes face query, laws
  and complete device reduction. The old event retains its raw integral scope;
  candidate checked end-to-end latency is also recorded. The other wall variants
  have numerical gates and recorded timings, without a separate throughput claim.

Both allocations together must fit 2 MiB, enforced at compile time; the new
packet alone is below 512 KiB. No stack limit, device setting, tolerance, leaf
cap or owner capacity changes. Device-wide free-memory readings explicitly
include context and both kernels' runtime backing, beyond owned buffers.
The full process has a 120-second guard/CTest limit. Performance failure is
retained evidence and triggers diagnosis, not increased workers or capacities.

Root registration: library `robo_dyna_source_nodal_wall_fixture` contains
`SourceNodalWallFixture.cpp`, links `robo_dyna_source_contact_force_fixture` and
`tl_nodal_wall_contact`. Executable `robo_dyna_source_part_nodal_wall_check`
contains `SourceNodalWallCuda.cu`, the unchanged `SourceContactCudaFixture.cu`,
`source_part_nodal_wall_check.cpp`, `source_part_nodal_wall_cuda_check.cpp` and
`source_part_nodal_wall_cost_check.cpp`. Link the fixture, existing wall
tessellation/canonical-artifact libraries, CUDA runtime and `GTest::gtest`.
The first host test translation unit owns main with positional arguments
`readiness.json wall.manifest.json`. Retain existing strict FP64 flags and add
`--expt-relaxed-constexpr` solely for standard-array device accessors. This does
not relax floating-point arithmetic.

The first actual execution passed all five numerical, coverage and rollback
functions. The cost function failed its unchanged 2 ms median threshold:
median 3.405312 ms, maximum 3.850144 ms, versus the contemporary unchanged
integral median 449.362762 ms (131.959349x). The maximum and speedup conditions
passed; checked candidate end-to-end median was 3.458890 ms. Combined explicit
device storage was 1,903,984 bytes. This first candidate failed the performance
gate. See the [guard report](../../../crash-work/reports/source-nodal-wall-tests-1.json),
[six-function XML](../../../crash-work/reports/source-nodal-wall-xml-1/robo_dyna_source_part_nodal_wall_check.xml)
and [retained first-execution source/runtime manifest](../../../crash-work/checkpoints/source-nodal-wall-first-execution-1/manifest.json).
The manifest preserves the original binary and every discovered workspace
dependency before adding the [optional phase profiler](NODAL_WALL_PROFILE.md).

The prepared-query candidate subsequently passed all six source functions,
including the unchanged median <=2 ms, maximum <=5 ms and speedup >=20x
conditions. The original100 coherent profile measured median 0.984992 ms,
maximum 0.991136 ms and checked end-to-end median 1.078937 ms, including host
preparation and its extra upload. The contemporary unchanged integral median
was 428.126343 ms (434.649551x). Explicit allocations were 459,376 bytes for
the candidate and 1,522,464 bytes for the baseline: 1,981,840 bytes together,
under the original 2 MiB cap. No physics, worker count or acceptance budget
changed. See [configure](../../../crash-work/reports/prepared-wall-source-configure-1.json),
[build](../../../crash-work/reports/prepared-wall-source-build-1.json),
[guard](../../../crash-work/reports/prepared-wall-source-tests-1.json) and
[six-function XML](../../../crash-work/reports/prepared-wall-source-xml-1/robo_dyna_source_part_nodal_wall_check.xml).
The six owning host-query functions also passed their [separate gate](../../../crash-work/reports/prepared-wall-query-tests-1.json).

Numerical checks retained the original/flip/subdivide finite walls and added
complete fallback for a last NaN, degenerate or overflow face, full-result
preservation and clean retry, plus successful one-ULP off-plane fallback.
The coherent nodal-versus-integral model differences remain exactly the earlier
reported 0.017875646388936427 N and 2.3966267184974635e-6 J. These are distinct
discrete contact models; query acceleration did not change their difference.
The [retained profiler snapshot](../../../crash-work/checkpoints/source-nodal-wall-profile-1/manifest.json)
preserves the measured diagnosis preceding this change. Passing this bounded
prescribed contact gate is not a whole-vehicle throughput measurement.

These prescribed comparisons and cost gates do not admit connected owner
dynamics, source mass/formulation equivalence, finite-edge crossing, friction,
thickness/director offsets, folding self-contact, or a Yaris crash trajectory.
