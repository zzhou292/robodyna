# Immutable rigid-PART startup aggregate

`NodalRigidPartAssemblyModel` retains a complete immutable coefficient ledger
and an exact owned clone of the supplied rigid source topology. It accepts
existing qualified SI coefficients only. A member without a supplied producer
rejects; a supplied zero-J member stays zero-J. Covered nodes do not establish
complete source coefficient coverage, dependent DOFs or owner admission.

The fixed policy sorts original NIDs separately inside each PART and extra set.
The primary position is the PART-only mean of represented SI coordinates in
that order. Literal topology order, source IDs, merge directions and excluded
plain-rigid membership remain available and participate in exact identity.
`RootForDomainNode` uses a sparse retained mapping; unused domain nodes remain
outside these roots. No generated primary is inserted into the physical domain.

Every original body keeps one native source-unit primary regularizer and one
raw PART/extra result. The implementation reuses `PrepareAssemblyRawBody`,
including INIRBY's extra-node center reset. It then performs source-directed
raw merges and exactly one `FinalizeAssemblyRawBody` per final root. No early
principal correction, replaced centroid or copied inertia solver is introduced.
All results publish together after every map, body, merge and finalization passes.

The ledger's authoritative nodal mass/native-total scalar J drive the aggregate.
Its shell physical/added, TYPE25 property, TYPE13 native-total/added and additive
point-mass partitions remain available through the retained ledger. Physical J
is never reconstructed by subtracting rounded values. Primary mass/J and final
principal correction are separate regularization channels. Root tensors are
constraint aggregates, not new additive nodal contributions.

This deterministic SI member traversal differs from native source-set traversal
and native-unit centroid conversion. Tests compare exact named identity fields,
independent native raw/merge packets, and binary128 mass/COM/tensor calculations.
The latter use operation-count bounds, absolute signed tensor terms and explicit
transported COM error; they do not treat cancellation-relative tolerance as a
proof. Eigen axes are an equivalent tensor factorization, not native VALPR bits.
The native nonlinear principal correction is checked separately.

The budget includes the complete retained coefficient backing once, an exact
topology clone, one arena of member/original/root/lookup records, clone views and
identity scratch, and transient mass-point packets. Counts precede input-member
reads and allocations. The public cap is 1 GiB / 1024 bodies / 16384 members; existing
runtime capacities do not change. Tests include late overflow and retry, missing
coefficient/identity, signed-zero identity, caller lifetime, exact byte caps and
a synthetic 22-primary/two-merge/20-root model with one 796-member root.

The separate original point-mass source gate retains 155 original literal rows,
including 54 rigid extras. This model's capacity fixture is not the original
22-body aggregate. Actual source admission waits for all required original rubber
coefficient producers under the explicit demo HEPH24/S6Z formulation mapping.
The original hourglass override instead resolves Isolid1 and remains separately
recorded in the case/source policy; this model does not choose that mapping. No original
rubber mass, force, runtime or complete-vehicle claim follows from these tests.

Standalone CMake defaults to host values only. `RIGID_PART_MODEL_NATIVE=ON`
reuses unchanged existing authenticated INIRBY/ADMAS5 and correction wrappers;
no reference code enters the production target. Root owns Fortran/native and
larger source gates. Author host checks use one CPU and 512 MiB.

Root qualification passes6 host and2 native numerical functions plus2 donor
identities in `rigid-part-model-root-tests-1`. Owning Bazel builds pass in
`rigid-part-model-owning-bazel-build-1`. The complete synthetic model retains
433752 B/startup497376 B. These are startup/model gates, not live vehicle dynamics.

The solid-ledger integration gate `solid-rigid-coverage-root-tests-1` passes
7 host and3 native functions plus2 donor identities. Each family separately
provides all coefficients for a solid-only rigid body with zero nodal J; its
raw mass/center/tensor matches the independent native wrapper. A missing
producer rejects atomically and the same model retries with the real ledger.
The shared `HasCoefficientProducer` predicate is used by both ledger coverage
and rigid admission, preventing an outdated list from dropping solid-only nodes.
Owning rigid-model and ledger targets pass in
`solid-rigid-coverage-owning-build-1`.
