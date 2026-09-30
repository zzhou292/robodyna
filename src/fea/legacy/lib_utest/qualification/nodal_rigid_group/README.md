# Nodal-rigid startup, recurrence packets and owner qualification

The startup model prepares disjoint, fully supplied source nodal-rigid groups.
Pure packets evaluate force/moment transfer, anisotropic angular acceleration,
principal-frame recurrence and member drift. The optional owner adapter now
composes those packets with the existing physical-node accepted/trial slabs.
See `lib_src/solvers/NodalRigidGroups.md` for its API, phase and limited scope.

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
claim is made by the stateless-math tests about native `ROTBMR`, `RGBODV`, first
half kick, finite-rotation drift or transaction publication. Separate packet,
phase and owner gates below cover those additional responsibilities.

The native directory retains nine complete pinned original files and twenty
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
See `lib_src/constraints/NodalRigidGroupStepMath.md` for that scope. All 12 initial packet tests pass alongside the 21 startup/stateless tests in six
host/native/CUDA targets on the RTX 5090 (2026-09-10). The owning Bazel targets
also build. Workspace evidence:
`crash-work/reports/nodal-rigid-step-{configure,build,tests}-1` and
`nodal-rigid-step-owning-bazel-1`. No owner integration is claimed.

The independent recurrence review subsequently found a floating-point grouping
difference when large lever-arm moments nearly cancel a small applied couple.
TLf4e9a03 preserves the native left-associated expressions on all three axes.
All36 rigid functions now pass, including three new cancellation tests and
an extended intermediate-overflow regression. Evidence:
`crash-work/reports/nodal-rigid-cancellation-{build,tests}-1` and
`nodal-rigid-cancellation-owning-bazel-1`. Earlier summaries saying37 included
the extended existing test as a new case; retained GTest output confirms36.

The new `nodal_rigid_phase_native_check` retains source fresh duration and kick
statements with a separately authored fixed-step selector. It checks unequal
durations and 32 evolving force/frame/member packets, including sensitivity to a
wrong first full kick and premature frame update. Source scheduling context is
recorded in the workspace `planning/NODAL_RIGID_STARTUP_PHASE.md`. Wrapper parity
does not establish execution of the full native engine.

Configure `TL_NODAL_RIGID_OWNER_CHECKS=ON` for `nodal_rigid_owner_check` and
`nodal_rigid_owner_native_check`. The former checks initialization, budgets,
immutable diagnostics, bypass rejection, first/later rollback, final-member
second-group overflow, new-spin bounds, and sparse source membership at node
2047. The latter compares 64 actual CUDA owner steps against the independent
native schedule and native force/frame/member arithmetic, with two groups,
changing/off loads, force/couple reactions and an ordinary free node. All group
values publish through the sole nodal slab swap. The owning Bazel CUDA target
is `//lib_utest/qualification/nodal_rigid_group:nodal_rigid_owner_check`; native
Fortran comparisons use CMake. These new runtime gates must pass before R3a is
treated as qualified. This change does not admit shell/contact coupling or
aggregate rigid-group energy reporting.

The separate value-observation targets are `nodal_rigid_observation_check`,
`nodal_rigid_observation_native_check` and `nodal_rigid_observation_cuda_check`.
See `lib_src/constraints/NodalRigidObservation.md` for explicit midpoint/lagged
frame phases, native member and aggregate K, primary/correction partitions,
applied versus reaction kick work, stable replacement deltas and arithmetic
budgets. These value functions do not attach to owner/common publication or
reconstruct native collocated output. `NodalRigidCollocatedObservationDesign.md`
records the distinct force-stage input and remaining qualification gate.

## Force-stage collocated value gate

`nodal_rigid_force_stage_check` and `nodal_rigid_force_stage_native_check` qualify
the separate pre-kick + actual A/AR + updated-axis observable. Native RGBCOR
fragments now have a separate nonzero-DT1 wrapper with both member loop branches;
the existing zero-duration wrapper and stored observation tests remain intact.
The optional `nodal_rigid_force_stage_cuda_check` needs TL_NODAL_RIGID_CUDA_CHECKS.
This adds no owner capture and makes no complete native global-energy claim.
# Native-total-only inertia attribution

`InitializeNativeTotal` admits physical assemblies whose source provides a
rotational inertia contribution without a physical/added split. The caller
supplies that exact contribution in `unpartitioned_native_inertia_kg_m2`.
All three attribution channels must agree with the independently supplied
native total within the existing roundoff bound. Tensor construction and
rigid dynamics continue to use native total J exactly once. The older
initializers still reject nonzero unpartitioned contributions.

The group model and kinetic observations retain this third channel explicitly;
it is neither physical nor added inertia. The new host controls compare identical
total-J tensors and kinetic totals across attribution changes, reject missing,
negative, nonfinite and double-counted evidence, and verify rejection/retry.
The bounded root run `rigid-native-total-root-tests-1` passes all 16 existing and
extended host/native CTests. Actual beam/plain-rigid CUDA binding is qualified
separately by `beam18_resident`.
