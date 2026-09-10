# Resident mixed shell collection gate

This gate expands fixed resident capacity, not mechanics or temporal
admission. All three new CUDA functions and 20 existing mixed/T3 functions pass. The sole nodal owner admits at most 128 nodes; each typed batch uses
the shared 128-parent/128-node limits and one bounded device allocation with two
history/cache slabs. A joined binding admits at most 128 parents **in total** and
both families must be present. The existing serial arithmetic/order is retained.
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
