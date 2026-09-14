# Isolated accepted self-contact force/STI assembly

`SelfContactForceAssembly` retains the exact
`SelfContactActiveUseBinding` authority and authenticates its retained physical
ledger, PART/plain binding and complete CIN roster against one fresh owner.
It accepts only a bounded caller batch of already discovered/resolved
`AdmittedVertexFace` or `AdmittedEdgeEdge` events. It adds no discovery,
broadphase, crossing query,
candidate geometry, app setup, contact history or final participant receipt.

Each VF event is reauthenticated against the retained parent/vertex/facet,
weighted maps, support roles, activity identities, reference thickness and
positive certified directed area. Each EE event regenerates both exact
edge-use ordinals and weighted edge points, authenticates the canonical
`FixedTriangleEdgeEdgeKey` against retained endpoint provenance, and requires
the positive symmetric directed edge-point certificate. Canonical fixed-feature key then source order
defines event arithmetic. Duplicate feature/source identities reject. The represented
coefficient is derived internally as the checked outward product
`stiffness_per_area_n_m3 * admitted_force_area_m2.value`; callers cannot supply
pair stiffness, effective mass, line area, damping, friction or approximation
radius.
An authenticated zero-distance EE area does not invent a force direction:
surface-pair evaluation returns `ZeroDistance`, the event batch fails closed,
and candidate force/STI publication does not occur.

All force, potential, moment and represented-majorant arithmetic executes in
CUDA kernels on the actual owner stream. Every event has a private
`SurfacePenaltyPacket`. A complete host incidence is sorted before upload and
one CUDA thread owns each touched node. It copies and validates all six incoming
force/couple channels plus current CIN translational STI, then adds full contact
force and diagonal majorants in canonical incident-event order. A separate
check precedes publication of force XYZ and translational STI. Couples and
rotational STI are never written. No floating atomics are used.

## Exact storage forecast

For configured event capacity `E` and physical owner node count `N`:

- event capacity: `E`
- checked incidence capacity: `8*E`
- touched-node capacity: `T=min(N,8*E)`
- staged channel values: `6*T`
- one identically laid-out host arena and one device allocation contain, in
  `BoundedArenaLayout` alignment order: `E` events, `E` packets, `8E`
  incidence records, `T` node ranges, `E` event statuses, `T` node statuses,
  `6T` doubles, `T` STI doubles, one control and one diagnostic.
- `host_arena_bytes` and `device_bytes` are the exact resulting aligned extent;
  `device_allocations` is exactly one.
- `startup_scratch_bytes` is the existing complete physical owner proof
  (`13*N + 2*N + 2*cin_rows + 1` doubles with arena alignment).
- `startup_host_bytes` is the checked sum of retained active-use payload,
  module/arena retained payload and that temporary proof. It is a payload
  forecast, not allocator, CUDA runtime or RSS accounting.

No attempt performs `cudaMalloc`, host allocation, capacity growth or
result-array readback. Only the fixed control and diagnostic records are read
back after the owner stream completes.

## Author gate

Run from the workspace root under the mandatory shared lock:

```sh
python3 Total-Lagrangian-FEA/tools/run_bounded.py \
  --report crash-work/reports/self-contact-force-sti-author.json \
  --lock crash-work/reports/connection-author.lock \
  --cpus 1 --min-available-gib 1 --max-rss-gib .5 --timeout 360 -- \
  bash -c 'cmake -S crash-work/worktrees/self-contact-force-sti/lib_utest/qualification/self_contact_force_assembly \
    -B crash-work/build/self-contact-force-sti-author \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_CXX_FLAGS_RELEASE="-O0 -DNDEBUG -g0" &&
    cmake --build crash-work/build/self-contact-force-sti-author --parallel 1 &&
    ctest --test-dir crash-work/build/self-contact-force-sti-author --output-on-failure'
```

This runs only pure host area/stiffness, canonical sorting/incidence, duplicate,
exact-cap and alias values; source proof; and public/host/CUDA-shaped syntax.
It invokes no NVCC, GPU, Fortran or full V5 target.

## Root CUDA gate

The default-off root fixture composes the real physical publication owner,
PART/plain execution authority and CIN transfer. Run only on the root CUDA
executor:

```sh
cmake -S lib_utest/qualification/self_contact_force_assembly \
  -B <root-build> -DSELF_CONTACT_FORCE_CUDA=ON \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_CUDA_ARCHITECTURES=<root-arch>
cmake --build <root-build> --target self_contact_force_cuda --parallel 1
ctest --test-dir <root-build> -R '^self_contact_force_cuda$' \
  --output-on-failure
```

The authored CUDA fixture covers exact caps/allocation stability; ordinary,
partial-axis and fully fixed owner masks; shared-node incidence; equal/opposite
resultant, global moment and independent virtual-work checks; initial half-kick
and ordinary interval phase identity; preservation of incoming force/couple/
STI; exact canonical results under input-event permutation; actual PART/plain
and CIN-master support/transfer; independent dense master force, moment and
STI enclosure with an actual surface CIN secondary rejected and unchanged;
committed partial/full fixed reactions; duplicate, stale,
foreign, same-body-excluded and CIN-secondary rejection; final-event NaN,
node-sum overflow, complete rollback/retry and stable allocations. CUDA
execution remains a root-only gate. No energy-conservation claim is made.
