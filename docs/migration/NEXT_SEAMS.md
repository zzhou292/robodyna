# Next module boundaries

These are the next dependency extractions after the neutral mechanics and generic
visualization work. Canonical class names and relocated files do not establish
independent FEA/MBD linkage. Current qualification is recorded in
[EXECUTION.md](EXECUTION.md) and [RENAME_QUALIFICATION.json](RENAME_QUALIFICATION.json).

## Completed boundaries to preserve

The generic visual split is complete. Geometry, visual values, ordinary visual
models and object services have their own native owners. FE attachment and update
definitions live in `src/fea/visualization/LegacyVisualAdapter.cpp`; generic model
update uses the retained per-model updater. The generic owners have passed actual
header/link/compile-boundary checks with FEA enabled, without concrete FE or body
implementations. Keep the focused attachment, copy, update, archive and viewer
regressions when extracting the next boundary.

The actual base body and mesh implementations now live in
`robodyna::mbd::RbBody` and `robodyna::fea::RbMesh`. The actual System/NSC/SMC
family lives in `robodyna::simulation`, with implementations in
`src/simulation/composition`. Legacy headers provide reverse aliases. These
implementations still compile once in the retained mixed backend; public `:api`
targets still depend on that aggregate. Private implementation-header providers
and the thin System forward target avoid introducing a build cycle. Do not make
those private providers depend back on the public aggregate API.

Preserve the original numerical behavior and archive identities. The visual
archive still represents ordinary shapes; the existing FE-shape omissions are
not physical restart support. Rebuild owned consumers when C++ layout changes;
source/archive compatibility does not promise compatibility with old binaries.

## 1. Extract the specific system services used by participants

`RbMesh.cpp` still accesses concrete System initialization/update flags, thread
settings and gravity. Nodes and elements receive the concrete system during
initial setup. `ChPhysicsItem` also stores the parent system and participates in
its offset and initialization lifecycle.

Inventory these exact operations, then introduce the smallest shared service
contract that removes the concrete dependency. Preserve model invalidation,
initialization order, gravity sampling, thread policy and DOF offsets. Do not
create a second state owner or clock, or route GPU batches through CPU vectors
and per-node virtual calls. Keep thin forward declarations available to consumers
that only need pointers; they should not acquire the complete backend as a side
effect of including a public name.

Gates: unchanged setup/invalidation behavior, repeated add/remove/setup cases,
gravity and assembled residual parity, and actual transitive header/link checks
for the extracted service owner. Existing native System and frozen archive tests
remain required. A header-only interface without a working domain consumer is
not completion of this step.

## 2. Keep mixed assembly above FEA and MBD

`ChAssembly` owns concrete body, link, shaft and mesh collections. Its traversal,
offset assignment, injection into the system descriptor and archive ordering are
part of current behavior. Keep mixed composition above the peer domains while
separating domain contributions at their existing assembly boundaries. Do not
replace the common assembled solve with mandatory partitioned stepping merely
to make directory dependencies appear independent.

Gates: preserve collection/iteration order, DOF and constraint offsets,
mass/force/Jacobian contributions, reactions and frozen archive bytes. Exercise
a flexible beam attached to a jointed rigid body through the same assembled
solve, including action/reaction moments and constraint work.

## 3. Separate contact algebra from domain response and reporting

`ChContactContainer` includes concrete body definitions and dynamically casts
contact participants to bodies for reporting. Tuple algebra also depends on
contactable interfaces. Extract neutral algebra only where its actual header
and link closure is neutral; keep body/FE force response and reporting in explicit
domain adapters. Reuse the current contact algorithms and solver policies.

Gates: NSC/SMC coupons, FE–FE and FE–body contact, equal/opposite force and moment,
report ordering and counts, and unchanged constraint/contact work. Do not disable
FEA, omit a contact participant or drop a reporting feature to obtain a smaller
link closure.

## 4. Finish domain-specific loads and prove independent execution

Split FE nodal loads/builders from their concrete body and motor attachment
helpers. Keep neutral frame constraints distinct from concrete coupling adapters.
Then execute genuine MBD-only and FEA-only targets without the opposite domain's
concrete implementations or a hidden path through the aggregate. Check actual
header, compile and linker ownership as well as runtime results.

The qualified CUDA vehicle backend retains FE nodes, rigid-region state and CIN
transfers in one owner and one publication. Preserve that whole backend and its
single advancement throughout this work. The renamed CPU System family is not
evidence that its execution has become CUDA, or that the two backends share a
numerical stepper. Keep their execution profiles explicit.

Complete the independent-domain gates and coupled solve regression before
expanding module extraction against [CAPABILITIES.md](CAPABILITIES.md). Retain
existing CPU and CUDA implementations until their replacements are separately
qualified. Use small focused cases and the short vehicle archive comparison;
documentation or ownership changes alone do not justify rerunning the long crash.
