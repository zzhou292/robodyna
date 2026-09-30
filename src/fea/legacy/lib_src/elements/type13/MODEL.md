# TYPE13 immutable startup model

`Model::Initialize` gives the existing qualified TYPE13 startup values durable
ownership for later coefficient and resident integration. It accepts one explicit
working-unit node table, source property declarations, and source-ordered beam
connections. It calls the existing `InitializeProperty` and `InitializeElement`;
it adds no material, frame, mass, force or integration equation.

`ModelNode::global_node` names the future common owner index. A source node used
only as N3 may carry `SIZE_MAX`; each actual endpoint must name a real index.
N3 remains part of source/reference identity and affects native frame selection,
but receives no endpoint coefficient. It can also be a real endpoint of another
beam, in which case the one node record must have an owner index. Distinct source
nodes may share coordinates. Distinct beam EIDs may share endpoints and each
retains its own native contribution; repeated EIDs or aliased node IDs/owner
indices reject. Every supplied source node and property must be referenced.

The node table avoids repeating coordinates in each connection. Reused
`SourceIdentityIndex` checks identities without changing source/reference order.
One `HostArena` owns exact active node/property/connection/startup arrays. Each
property keeps original scalar declarations and its four copied five-point
curves, plus the qualified pointer-free prepared `Property`. Borrowed curve
views are rebound to owned storage. Retaining raw scalar declarations matters:
two different tiny supplied inertias can resolve to the same native floor and
added-J bits, but must remain different source scopes. Models share only their
immutable storage, and `Matches` compares named input fields, not object padding.

`Endpoint(e, local)` returns an existing startup coefficient associated with
original EID/PID/NID and common-owner index. It performs no reduction or new
mass computation. Traverse connection then endpoint order when a later binder
adds these rows. `isotropic_inertia_kg_m2` is the native total and already includes
`added_inertia_kg_m2`; adding those two fields would count the floor twice. Raw
source inertia remains available in the property declaration. No mass goes to
an orientation-only node and no positive inertia is fabricated for other nodes.

Count/range/byte checks precede allocations and curve value reads. Hard bounds
are8192 beams,1024 properties,16384 local source nodes and1048576 owner nodes,
with a128 MiB simultaneous host payload cap. The forecast includes fixed
headers, a conservative shared-control reserve, the exact aligned arena, all
four temporary identity indexes and node/property-use arrays including their
shared-control reserves. It excludes caller-owned input, another retained model,
ordinary call stacks and allocator bookkeeping; it is not process RSS. Any
failed count, identity, last reference, allocation or native check leaves the
model unprepared and allows retry. A prepared model is immutable. Failed queries
leave caller output unchanged.

This model does not construct a native force history or an accepted cache. The
later resident participant must authenticate actual owner endpoint positions,
midpoint velocity/spin, step phase and original working-unit conversion before
calling the existing H1 recurrence. It must join the common publication and
mass scope. Source card authentication, tied attachments and source-to-owner
mapping are separate app responsibilities. Startup success is not a connected
vehicle admission.

Owning CMake checks live in `qualification/type13_model`: seven small host
functions and two optional original/native functions. The latter reuse the
original4442-beam/7494-source-node fixture and the existing native startup oracle
without copying donors. They verify all7493 endpoint owner indices, every
ordered EID/PID/NID/coefficient and native frame, N3 exclusion, complete scope
identity and last-row/exact-budget retry. Native comparisons keep the existing
startup gate's exact comparisons and source-to-SI operation order.

Root qualification on 2026-09-11 passes all seven host and two original/native
functions, both reused source-identity checks and the owning Bazel target.
Independent read-only review found no blocker in storage lifetime, N3 ownership,
coefficient interpretation or failed-initialization handling. Original model
owned payload is1,581,424 B; simultaneous startup payload is1,900,423 B.
Evidence is retained under `crash-work/reports/type13-model-root-*`, including
the initial failed Fortran-runtime link and its CMake language-scope correction.
No donor arithmetic or comparison tolerance changed.
