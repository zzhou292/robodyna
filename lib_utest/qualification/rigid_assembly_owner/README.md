# Prepared PART assemblies in the common nodal owner

This increment wires the already prepared `NodalRigidAssemblyBinding` into
`FENodalState`. Its compact source records preserve PART versus plain-group
namespaces, root order, source NIDs and indices, exact source coordinates, raw
M/J and the already finalized principal body. It does not rerun primary
regularization, raw merging or finalization. The prepared binding retains the
complete immutable coefficient ledger; the live owner copies only its compact
rigid records. All bodies use the existing accepted/trial slab and commit.

Use the distinct overload and explicit profile:

```cpp
config.rigid_limits = tl::fea::NodalRigidOwnerLimits::VehicleAssembly();
owner.Initialize(config, initial, inverse_mass, dofs, binding, optional_cin);
```

The default plain-group limits and `Vehicle()` retain their existing 64/256
and 1024-groups/8192-members/256-per-group contracts. `VehicleAssembly()` admits
at most 1024 groups, 16384 members, and 1024 members per group, with the same
8 MiB compact host budget. Actual owner node and device byte limits remain
separate. The host layout test covers the current census of 779 roots and
12991 members, the 1024-member boundary and overflow. It is not an actual
full-vehicle runtime test.

PART members may have nonnegative M/J, including literal zero. They remain
kinematically present and receive primary-driven motion and the qualified
reaction calculation. This uses the previously qualified dependent member
step policy; it does not invent an ordinary-node rotational DOF. An ordinary
solid-only node with J=0 instead retains the explicit absent-rotation policy.
The complete binding's ledger validates all supplied owner inverses, and its
domain validates every coordinate bit before device allocation.

Optional CIN must have the same complete domain and exact raw ledger M/J.
CIN masters and dependents cannot intersect any actual rigid member. Initial
inverse M/J is copied from validated input. Force-stage and readback code retain
zero inverses for zero-coefficient PART members, and readback checks that CIN
has not changed unrelated rigid coefficients. Existing shell/connector
`native_physical_coefficients` admission stays plain-only: this increment does
not yet admit complete mapped shell/solid/connector participants.

`NodalRigidGroupInfo` appends PART root count and plain-source identity; rigid
state and force-stage snapshots append source kind. Existing positional
aggregate initialization remains valid with the new fields defaulted. Old
plain-group arithmetic, reduction order and selector behavior are unchanged.

## Qualification boundary

Author checks: five new host functions and eighteen affected host functions
pass under one CPU / 512 MiB. New CUDA test source passes ordinary host syntax
checking; native source identity is checked independently. Fortran, NVCC,
actual GPU, owning Bazel and full runtime qualification belong to the root
runner and are **not claimed by this author freeze**.

The synthetic fixture uses real qualified solid18, shell, TYPE25 and literal
zero ELEMENT_MASS coefficient producers. It contains four PART members, a
plain two-member group with the same numeric source ID in another namespace,
an ordinary solid-only J=0 node and two disjoint CIN patches. It does not claim
original Yaris whole-model/source-card admission.

The CUDA/native gate contains five functions: source/DOF rejection with
preallocation retry, failure in the second body and late group readback
rollback, CIN epoch-zero/prepared/accepted inverse zeros and typed force-stage
capture, and two 64-interval mixed-body trajectories (with/without CIN). The
trajectories compare against the unchanged independently retained native
wrench, primary, frame, ordinary-member, two-member and schedule packets from
`nodal_rigid_group/native`, including zero-M/J reactions. Prepared body startup
values reuse the separately qualified assembly/binding; this gate is not a new
oracle for that startup. Nodal quaternion integration and CIN force transfer
retain their existing separate qualifications.

Root gate:

```sh
cmake -S lib_utest/qualification/rigid_assembly_owner -B BUILD \
  -DCMAKE_BUILD_TYPE=Release -DTL_RIGID_ASSEMBLY_OWNER_CUDA=ON \
  -DCMAKE_Fortran_COMPILER=GFORTRAN_LOCAL -DCMAKE_CUDA_ARCHITECTURES=120
cmake --build BUILD --target rigid_assembly_owner_host rigid_assembly_owner_cuda -j1
ctest --test-dir BUILD --output-on-failure
```

The CTest names are `rigid_assembly_owner_host` (23 host functions),
`rigid_assembly_owner_cuda` (five GPU functions, including two native histories),
and `rigid_assembly_owner_native_identity`. No donor or oracle equation changes.
Bazel owners are `//lib_src/solvers:explicit_nodal_state`,
`//lib_utest/qualification/rigid_assembly_owner:rigid_assembly_owner_host` (five
new host functions) and the sibling `:rigid_assembly_owner_cuda` (three GPU
functions; native trajectories are owning-CMake tests).

Affected root gates: existing `nodal_rigid_group` owner, two-member, vehicle,
prepared-snapshot and force-capture suites; `tied_cin_runtime` native/CUDA;
`rigid_shell_contact` coefficient admission. Original plain shell/connector
regressions must remain closed to the new assembly scope until the mapped
participant work is admitted separately.
