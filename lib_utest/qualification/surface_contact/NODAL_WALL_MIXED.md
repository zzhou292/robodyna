# Native T3 and mixed nodal-wall contributor gate

Source contract before execution. Extend the existing
`reference-area-lumped-nodal-wall-v1` contributor to one/two native Q4 and/or
T3 parents. This is a contact-contribution gate with actual native masses;
it does not admit mixed shell force-feedback dynamics, a full-part timestep,
source MAT024 behavior, or a 94-parent/117-node production owner.

The production change stays in `NodalWallContactModel.cpp`,
`NodalWallContactKernels.cuh` and public scope comments in
`NodalWallContactDevice.h`. No new participant, history, owner, clock, stream,
allocation or dispatch framework is introduced. The immutable host
`NodalWallWeights` already accepts prepared Q4 center-area and intrinsic T3
references, retains native parent identities/connectivity, and sorts parent
shares. Require exactly `(Q4CenterArea,4)` or `(T3Native,3)` before publication.
All four local-node loops use that validated arity. The stride-four share
storage stays unchanged; no T3 fourth share is evaluated or read, and its
unused parent-force output is zero from the existing complete result reset.

Q4's four iterations, numerical expressions and sorted-parent addition order
are preserved. The point law, finite-wall query/coverage, per-parent certificate
checks, unique-node/global reduction, all-destination staging, interval work,
identity and failure handling are unchanged. Contact shares remain A0/4 for Q4
and A0/3 for T3. Shared area is never applied as a second spring. Shell union
mass uses its own native Q4-then-T3 reduction, including T3 angle-weighted mass
and native total J; contact neither derives mass from its area nor reconstructs
J from partitions. The contact's sorted-parent reduction order is independent
of that native mass order.

The existing caps remain two parents, eight incident nodes, 64 owner nodes and
256 KiB total contact device storage. Pinned storage remains 99,384 bytes in one
allocation, with no per-step allocation; actual unchanged layout and allocation
are to be verified by root. The six owner allocations and mixed shell storage
remain owning-module concerns. The 128-parent/128-node host operation and prior
117-node prescribed source kernel are separate evidence, not a capacity bypass.

Focused host/CUDA tests are authored separately. They must include a genuine
T3 without global node zero (the unused fourth slot must add no spring),
one/two scalene T3s and a mixed shared-edge union; actual native mass and full
host-certificate parity; half/full-kick, drift, potential and wall impulse
truths; finite-wall seams/tessellation and current edge-on geometry; late
second-parent accuracy/mass/addition failure, unchanged accepted/output bytes
and clean retry. No coupled shell trajectory is inferred from contact-only
spring checks or disposable contact assembly.

The ordinary, source-scale and failure-only fixtures retain the distinct
tuples and inherited budgets documented in
[the frozen native test contract](NODAL_WALL_NATIVE_TESTS.md) and
[the prior owner contract](NODAL_WALL_OWNER.md). Source-scale probes stop at
assembly/readback; only the ordinary tuple advances the contact-only owner.
Global uncertainties remain outward sums. The host/device 2e-12 dimensional
comparison and independent interval ledger scales remain unchanged. Root owns registration,
source-map revision, builds and numerical execution. Prior Q4 host/device and
CW0 coupled regressions are required after this extension.
