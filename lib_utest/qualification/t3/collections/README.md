# Resident mixed shell collection gate

This gate expands fixed resident capacity, not mechanics or temporal
admission. All three new CUDA functions and 20 existing mixed/T3 functions pass. The sole nodal owner admits at most 128 nodes; each typed batch uses
the shared 128-parent/128-node limits and one bounded device allocation with two
history/cache slabs. A joined binding admits at most 128 parents **in total** and
both families must be present. Each parent's arithmetic, force assembly order
and diagnostic reduction order are retained.
The publication coordinator reduces global native kinetic energy exactly once.
No production allocation occurs after startup. Actual allocation bytes are test
properties and must be measured by the first serialized host/CUDA build/run.

The numerical fixture was fixed before execution: 117 nodes on a
13 by 9 grid with 0.125 m edges, the first 88 of 96 square slots as Q4s and the
last eight split into 16 native triangles. Material is the retained MB1 fixture:
density 1024 kg/m³, thickness 1/32 m, E = 2 MPa and Poisson ratio 0.3. The fixed
step is 2^-13 s, the prior MB1 step divided by eight to match the edge scaling.
Four intervals apply one known pulse then three free kicks. Initial x is the
reference, v/omega and material history/cache are zero, and q is identity.

The pulse supplies force_x = m*(0.25 + 0.5*x) and couple_y = J*(0.125 + 0.25*y).
All other components are zero. Device loads are pointer-backed, so raising the
owner cap cannot overflow CUDA's small by-value argument packet. The old
NodalTemporalFixture deliberately retains its 64-node packet.

Native startup/force comparisons reuse the qualified per-field 2e-12 dimensional
budgets. Owner kick/drift uses the retained 2e-13*(1+|expected|) arithmetic
budget. Both omitted-family controls must exceed 32 times that owner budget;
a weak control fails this gate rather than changing its load. Global kinetic
and work use 256*epsilon times absolute operand sums plus 1e-12*1e-6 J.
Every native result and every ledger must pass before issuing the joint receipt.

The final Q4 parent (index 87), final T3 parent (index 15), and physical node 116
exercise the enlarged ranges. A deliberate final-parent failure in either
family follows the other family's successful candidate; accepted nodal fields,
all histories/cache values and common diagnostics must survive. A fresh attempt
must agree exactly with the clean trajectory. Count/union mass rejection occurs
before any accepted publication. No contact, long response, impact or 117-node
vehicle-part dynamics qualification is claimed by this short collection gate.

Passing runtime: nine production device allocations total 817,119 B: owner 48,223,
QEPH 440,752, T3 323,968 and shared kinetic scratch 4,176 B. Test-only pointer
loads add 6,144 B. Across the three new functions,704 QEPH and 128 T3 native
cell intervals pass, including every retained force/history field. The omitted
Q4/T3 contributions exceed the arithmetic budget by 112,999/104,824 times.
No tolerance, loading, timestep or mechanics arithmetic changed after execution.

Evidence is `crash-work/reports/resident-shell-collection-{build,tests}-*.{json,log}`
and tests-1 XML. The actual build-2 took 28.472 s with 823,590,912 B sampled RSS;
tests-1 took 0.988 s. Initial build-1 had no generated target yet; configure-1
rejected modification of the pinned port CMake entrypoint. Registration now
uses the existing mixed feature's CMake extension, preserving that pinned file.
Configure-2 and build-2 pass. Both source-review macro fixes preceded compilation.

Twenty affected standalone QEPH and joint-wall functions also pass under
`shell-contact-collection-qeph-tests-1/`; six owning Bazel targets pass under
`shell-contact-collection-bazel-1.*`. Native references remain test-only.

The enlarged owner also passes all seven accepted Chrono output functions in
`shell-collection-output-tests-1.xml`. Both full coarse wall rebounds preserve
all scientific fields exactly, with only changed allocation/runtime metadata
excluded; see `wall_response/RESULTS.md`. No original-source dynamics or video
is implied by these capacity and compatibility results.

## Parallel candidate evaluation

Each QEPH/T3 candidate now evaluates one parent per CUDA thread in a single
128-thread block. A shared status array and an ascending serial scan preserve
the lowest failing parent and existing diagnostic reduction order. Assembly
remains deterministic, and failed candidates never publish accepted histories.
This adds no device allocation and does not change the per-parent force math.

One additional collection test exercises simultaneous first/last parent failures
in both families, preserved accepted state and exact retry. It passes with all
23 existing mixed/T3 functions and 20 affected QEPH/contact functions. Both
owning Bazel batch targets and the actual source-part native gate also pass.
Reports are `parallel-shell-candidates-tests-1`, `parallel-shell-qeph-tests-1`,
`parallel-shell-bazel-1` and `parallel-shell-source-native-1` under
`crash-work/reports/`.

For the complete original 117-node/94-parent elastic source experiment, the
parallel run takes 99.327 s versus 331.503 s for the serial baseline (3.337x).
All 778 scientific files, totaling 42,954,314 bytes, match exactly. Only elapsed
time and its inventory metadata are excluded. See
`source-part-elastic-parallel-parity-1.json`. The same physical experiment also
passes h/h2/h4 comparison at 257 common times, including genuine chord-length
change of about 0.277 mm. These results qualify this small experimental elastic
part, not vehicle-scale throughput, source plasticity or impact.


## Strided active-parent evaluation (2026-09-10)

QEPH and T3 candidates now use 64 workers with active-parent striding, followed
by the unchanged ascending status scan and numerical reduction. Storage and
admission remain at their existing resident128 bounds; the launch no longer
requires as many workers as the storage capacity. This is coverage preparation,
not a throughput or larger-assembly admission claim.

The existing 88-Q4/16-T3 fixture covers multiple QEPH iterations. The same
fixture and native oracle are also compiled for32 Q4/80 T3/90 nodes, exercising
the T3 tail with four forced/free steps and every native history/work comparison.
Failure at T3 parent79 preserves all accepted state and exact clean retry.
All four collection/mixed/T3 groups pass (`shell-stride-tests-1`), followed by
the actual original-part plastic engine and resident plasticity tests
(`shell-stride-plastic-tests-1`). Existing first/last simultaneous-failure tests
continue to check deterministic lowest-parent diagnostics.
