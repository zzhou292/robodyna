# Nodal-rigid startup and stateless mathematics

This module prepares disjoint, fully supplied source nodal-rigid groups and
evaluates force/moment transfer and anisotropic angular acceleration. It has no
runtime state owner, clock, constraint projection or accepted trajectory.

`NodalRigidGroupModel` copies source IDs, global indices, coordinate bits and
all native mass/inertia values into active-size host storage. The admitted
global-node count is independent of current shell/owner capacity. Startup
limits bound groups, members and owned payload including temporary identity
indices. No global-node-sized map is allocated, and no runtime allocation or
CUDA memory belongs to the model.

The source unit contract is explicit: member input is SI, while generated
primary `1e-20` mass and diagonal inertia values are in declared source units.
Original t/mm/s input therefore uses scales `{1000,.001}`. Native total nodal J
remains authoritative. Its separately supplied physical/added partitions may
differ in their final sum by up to 64 binary64 epsilons relative to the larger
total; the code never substitutes that sum for native total J.

The raw tensor includes parallel-axis terms and full native scalar shell J.
Eigen provides a right-handed principal frame with ascending eigenvalues; its
frame is an equivalent tensor factorization, not a claim of bitwise agreement
with native `VALPR`. Source `Ispher=2` correction retains its asymmetric `<=`
outer and `<` per-axis thresholds, including the outer native `1.E-3` single
literal promotion versus binary64 inner `EM03`, and records every added principal/tensor
inertia. Regularizer and correction ledgers must be integrated explicitly in
any future physical energy/mass reporting.

`NodalRigidGroupMath.h` contains allocation-free host/device functions. Wrench
reduction includes each member couple and its force's moment arm. Angular
acceleration includes anisotropic Euler terms in a supplied current principal
frame. It does not update that frame or choose its temporal phase. Thus no
claim is made about native `ROTBMR`, `RGBODV`, first half kick, finite-rotation
drift or transaction publication. Those remain the next integration gate.

The native directory retains six complete pinned original files and fourteen
byte-exact arithmetic fragments. `verify_sources.py` checks source SHA-256,
Git blob identities and exact source line extraction. The Fortran wrapper
supplies bounded input packets around original inertia, correction, force
moment and gyro statements. The wrapper context is separately authored;
these are focused arithmetic oracles, not execution of the complete native
rigid-body engine. Independent long-double tensor, virtual-work and world
angular-momentum-equation tests complement native agreement. A separate tiny-
mass case makes the generated primary's mass, COM shift and parallel-axis
inertia observable in an independent long-double calculation.

Standalone configuration:

```
cmake -S lib_utest/qualification/nodal_rigid_group -B NEW_BUILD_DIR \
  -DTL_NODAL_RIGID_CUDA_CHECKS=ON -DCMAKE_CUDA_ARCHITECTURES=120
```

Build/test targets: `nodal_rigid_group_check`,
`nodal_rigid_group_native_check`, `nodal_rigid_group_cuda_check`.
The owning production CMake include is `lib_src/constraints/NodalRigidGroup.cmake`.
Bazel owns production target `//lib_src/constraints:nodal_rigid_group` and host
test target `//lib_utest/qualification/nodal_rigid_group:nodal_rigid_group_check`.
Native Fortran and optional CUDA packet checks use the standalone CMake target.
Use the workspace resource guard for all actual builds and numerical jobs.
All 21 functions pass across the host, native and CUDA test targets on the
RTX 5090 (2026-09-10). The owning production and host-test Bazel targets build.
Workspace evidence: `crash-work/reports/nodal-rigid-startup-{configure,build,tests}-1`
and `nodal-rigid-startup-owning-bazel-1`. The independent review caught an omitted
native x-force assignment in the initial oracle extraction; the qualified
fragment includes lines 122–130, with the original source pin unchanged.
These gates qualify startup/stateless math only; constrained recurrence and
its common nodal publication remain separate integration work.

The separate pure recurrence packet targets are `nodal_rigid_step_check`,
`nodal_rigid_step_native_check` and `nodal_rigid_step_cuda_check`. They compare
frame/gyro/member acceleration/kick/drift arithmetic with explicit duration
inputs, including a proposed TL half-kick packet. They do not integrate an
owner, qualify donor engine startup timing, or publish a constrained trajectory.
See `lib_src/constraints/NodalRigidGroupStepMath.md` for that scope. All 12 new packet tests pass alongside the 21 startup/stateless tests in six
host/native/CUDA targets on the RTX 5090 (2026-09-10). The owning Bazel targets
also build. Workspace evidence:
`crash-work/reports/nodal-rigid-step-{configure,build,tests}-1` and
`nodal-rigid-step-owning-bazel-1`. No owner integration is claimed.
