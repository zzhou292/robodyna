# Explicit physical finite mesh-wall contributor

`NodalWallMappedContact` retains the complete immutable physical/rigid bindings
and borrows the actual common publisher, all six participant objects and owner.
It owns contact scratch only. The existing `NodalWallContactDevice` profile,
device layout and first-failure routing remain strict and unchanged by default.

The source gate checks every weight-sorted EID against its original QEPH/T3/QBAT
family index, ordered domain nodes and owning Q4/T3 area/share certificates.
Distinct coincident layers remain distinct. All incident source nodes require
actual free world DOFs with rotations present; other domain nodes may have
absent rotations. CIN secondary surface nodes are explicitly unsupported.
PART/plain surface nodes use actual aggregate M/principal J and the qualified
current force-frame reconstruction from authenticated accepted snapshots. No
member mass, inverse, rotational DOF or history is fabricated. Runtime activity
comes from the actual typed accepted/prepared owner histories, including virgin
and rigid-skin values; caller Boolean substitutes are not accepted.

Shared arithmetic preserves the legacy zero-damping force/potential order and
certificates. Mapped records deliberately have no free-mass rank-one row or local
velocity-first timestep. Accepted parent activity stays fixed through contact
force, candidate potential and drift/kick work. Newly removed parent potential
at candidate coordinates is reported separately. Inactive parents and nodes
have canonical zero records with their source identity retained. Contact force
and nominal STI additions validate in private staging before either destination
changes; STI enters the actual CIN assembly before its existing native transfer.
No generic free-DOF stiffness rows are submitted to the owner.

The outward rate screens the frozen **contact-only** ordinary-node / rigid-body
response (`h * sqrt(rate) < 1.6`). This is not combined structural/CIN step
admission. The current owner consumes/transfers STI/STIR but does not derive a
step bound from resulting current M/J and complete STI/STIR. A combined screen
belongs after native CIN transfer in the owner; this module does not reconstruct
or duplicate that state. The small loaded trajectories below qualify only their
explicit step and loads. Full original mesh-wall dynamics, joint closure and
long-run stability are not claimed by this first contact gate.

## Memory

Defaults retain the existing local device/host hard caps. Vehicle opt-in uses
2 GiB per local cap and an explicit complete startup reservation up to 8 GiB.
`Forecast` separates shared source payload, retained contact payload, temporary
host preparation and exact device allocation. Shared rigid backing is deducted
only through exact execution group/member backing identity. All extents precede
optional allocation. The physical source, caller-owned geometry/weights and
owner/other participant allocations are composed by their named owners; no
allocator, CUDA runtime/driver or RSS equivalence is asserted.

An allocation-free probe for 349,645 parents, 359,785 incident nodes, 372,435
owner nodes and 773 groups reports:

- Existing contact arena: 1,079,555,072 B.
- Mapped sidecar: 8,000,160 B.
- Exact device sum: 1,087,555,232 B.
- Existing host preparation alone: 1,081,569,196 B; the public mapped forecast
  additionally charges retained sidecar, complete mappings/activity/snapshots,
  temporary source index/positions/roles and initial-owner proof.

This probe reads no original source and allocates no full-count arrays.
`physical-wall-layout-author-2.{json,log}` records the bounded measurement.

## Owning qualification

Run from a fresh root-owned build directory:

```sh
cmake -S lib_utest/qualification/physical_mesh_wall -B <build> \
  -DPHYSICAL_MESH_WALL_CUDA=ON -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_CUDA_ARCHITECTURES=120
cmake --build <build> --target physical_mesh_wall_values physical_mesh_wall_cuda --parallel 1
ctest --test-dir <build> -R '^physical_mesh_wall_(values|cuda)$' --output-on-failure
```

Four host functions cover independent closed-form force/work, exact legacy
certificates across masses/velocities/signed contact, inactive share semantics,
and full-count exact sidecar byte caps/retry. Three actual CUDA functions reuse
the complete physical publication fixture (Q/T/QBAT, TYPE25, TYPE13, three solid
families, PART/plain and CIN): exact source/cap/alias checks and loaded rigid
contact with capture rejection/retry; late last-node STI failure with unchanged
forces and every accepted field; actual T3 one-point removal with same-mask work,
separate removed potential, exact retry and two post-removal attempts.

The shared fixture adds only default-off test options: one plain group can use
the T3-only surface node (the fixture weld is redirected to another admitted
endpoint), and the T3 failure strain can be selected for the removal control.
All prior fixture defaults remain unchanged. No native donor or mechanics
formula is changed. Rigid response has its separately root-qualified dense,
native-frame and CUDA oracle (9398f51); no new independent full contact oracle
is claimed here. Rebuild unchanged physical publication tests and existing
`utest_nodal_wall_model`, `utest_nodal_wall_weight_capacity`,
`utest_nodal_wall_arena`, `utest_nodal_wall_owner_cuda`,
`utest_nodal_wall_native_cuda`, and `utest_nodal_wall_collection_cuda` for the
shared preparation/point/kernel extraction. The full-count capacity test remains
root-only and need not repeat unless that regression detects a layout concern.

Author evidence: the owning CMake host target passes all four functions in
`physical-wall-cmake-values-2`; the first configuration exposed an unnecessary
Eigen header dependency, resolved by separating the allocation-only `Layout.h`
from physical publication storage. `physical-wall-values-1` also passed those functions;
`physical-wall-author-syntax-2` passed eleven host translation units;
`physical-wall-author-syntax-5` passed ten current host/test units and both new
and legacy CUDA-shaped C++ units in 8.272 s, with 375,058,432 B sampled peak RSS. These are not NVCC/device or runtime results.
The initial Clang CUDA syntax attempt failed on the installed Clang 14 C++ header
search (`cmath` unavailable); the failure is retained as
`physical-wall-cuda-syntax-1`. No native, CUDA or original-source run was made in
the one-CPU/512-MiB author lane. Root owns those remaining gates.

Root's first NVCC build passed, then all three CUDA fixtures stopped at the
second Q4 source reference: the reused reference was already prepared. The
follow-up resets that bounded value before each parent's single-use initializer.
The unchanged three-Q4 fixture covers the full loop and the late altered-T3
identity rejection. `physical-wall-tests-1` preserves the failing runtime evidence;
no force, area arithmetic, tolerance or legacy initializer was changed.
