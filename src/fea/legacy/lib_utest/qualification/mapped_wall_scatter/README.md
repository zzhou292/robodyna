# Mapped wall independent-node publication

The selected mapped wall already has a unique immutable compact-to-owner node
map. `BuildVehicleIncidence` rejects repeated global destinations during source
preparation. The new schedule reuses its existing device arena and sidecar:

1. Existing point/parent/global contact evaluation, rigid-response reduction and
   timestep check retain their arithmetic and order.
2. Independent nodes stage the actual current CIN translational stiffness plus
   contact stiffness, resetting their own `node_status` slot. One lane checks
   those statuses in compact order before any force staging.
3. Independent nodes privately copy all six force/couple channels, apply the
   existing `AccumulateNodalForces<1>` and unchanged interval addition/error
   operations, then a second compact-order status check completes.
4. Only after both checks succeed, independent nodes publish all six force/couple
   channels and translational CIN stiffness. Rotational stiffness is untouched.
   The final one-lane operation marks diagnostics valid or records the first
   numerical failure on the owner. Accepted-base copying follows this operation.

All operations share the existing owner stream. The reused point `node_status`
array has no live earlier consumer when these stages begin. Every stage resets
all statuses it may read, and a prior failure disables all subsequent stages.
`staged_force` retains its global six-channel indexing; `addition_error` and
`side.stiffness` remain compact indexed. There is no allocation/layout/forecast
change, floating atomic, or second owner. Numerical rejection leaves all owner
force and stiffness destinations unchanged; private failed scratch is not a
published result. Existing CUDA-error poisoning remains unchanged.

`NodalWallContactScatter.h` extracts the original copy/add/publish operations.
The legacy operator keeps its channel-first copy, compact-order accumulation,
and channel-first publication loops. Mapped publication changes only the order
of writes to disjoint node destinations. Zero-valued force and couple additions
are retained, including their native effect on signed zero.

The independent `FrozenScatter.h` retains the complete original StageStiffness,
Scatter and PublishScatter bodies from revision `4853da8`. Only the test
namespace and host/device annotation differ. `source-manifest.json` records the
complete original owning source hashes and the frozen header/shared arithmetic
hashes; configure and `mapped_wall_scatter_identity` verify the latter records.
No reference function calls the extracted production node helpers.

The small packet uses 137 compact nodes over a sparse, nonmonotonic 257-node
owner domain. Host checks reverse completion order. CUDA checks span three
blocks, compare every destination bit and successful error radius, and cover
signed zeros, cancellation, absent incident nodes, independent earlier/later
failures, stiffness-before-force priority, prior timestep rejection and retry.
A separate CUDA control compares the still-serial legacy operator to the frozen
implementation on success and late failures. The existing actual all-family
owner test now checks all eight destination arrays after late stiffness or
force failure, accepted-state preservation and a new attempt's successful retry.

## Owning gates

```
cmake -S lib_utest/qualification/mapped_wall_scatter -B BUILD \
  -DMAPPED_WALL_SCATTER_CUDA=ON -DCMAKE_CUDA_ARCHITECTURES=120
cmake --build BUILD -j1
ctest --test-dir BUILD --output-on-failure
```

Targets: `mapped_wall_scatter_host`, `mapped_wall_scatter_cuda`.
Tests: `mapped_wall_scatter_host`, `mapped_wall_scatter_cuda`,
`mapped_wall_scatter_identity`.

Affected gates: existing `mapped_wall_evaluation_cuda`,
`physical_mesh_wall_values` / `physical_mesh_wall_cuda`, and the legacy nodal-wall
surface-contact owner/collection CUDA tests. Owning production Bazel targets:
`//lib_src/collision:nodal_wall_contact_device`,
`//lib_src/collision:nodal_wall_mapped_device`,
`//lib_src/collision:nodal_wall_mapped`.

Author evidence: two host functions pass; four transformed CUDA-shaped C++
production/test translation units pass syntax under one CPU / 512 MiB. This is
not an NVCC compilation, CUDA execution, or measured speedup claim. Root owns the
three new CUDA comparisons, actual owner regressions and original loaded-prefix
runtime measurement. No donor mechanics or tolerance changes are part of this
slice.
