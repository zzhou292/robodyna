# Bounded self-contact broadphase adapter

This slice reuses the existing HydroelasticBroadphase sweep/overlap/filter body.
It adds an explicit adapter for immutable S0 `SelfContactSurfaceBinding` parent
maps and borrowed `VectorView` coordinates, with complete preallocated scratch.
It supplies a broadphase pair inventory only. No forces, owner clock, contact
history, self-contact runtime participant, original-vehicle admission or speedup
is claimed.

## Production contract

`SelfContactBroadphase::{Preflight,Initialize,Evaluate,pairs}` retains the exact
S0 source handle and uploads parent node ordinals/arity/reference half-thickness.
It neither copies coordinates into another solver nor reads immutable initial
activity as a runtime exclusion. The physical domain node count and supplied
strides must agree. Concurrent mutation/use of coordinates or the adapter is
outside this borrowed-lifetime contract.

Every T3/Q4 parent gets a directed-outward AABB around its current midsurface
corners, inflated by its authenticated **reference half-thickness**. The explicit
`LinearNodalEndpoints` mode includes both nodal endpoints; convex combinations
cover straight-line nodal interpolation of linear T3/bilinear Q4. This is not
rigid-arc sweep, CCD, an evolving-thickness model, current-map admissibility or
an intersection/contact certificate. An endpoint supplied with `Current` is
rejected. The original `ReferenceThicknessDistanceV1` policy remains separate
from later facet approximation and feature activation rules.

Null, legacy-default and per-thread-default stream handles are rejected. The
caller supplies a live stream and coordinate buffers through each synchronous
Evaluate. All actual adapter kernels, CUB operations and copies use that stream.
CUB storage-size queries precede allocation. The query:

1. Revokes the old published view, validates inputs and constructs bounds.
2. Reads the lowest invalid S0 parent before sorting any bounds.
3. Sorts axis keys and counts/scans **uint64** complete overlaps, then reads the
   full total before capacity admission or narrowing.
4. Fills admitted pairs and radix-sorts canonical unsigned keys, then synchronizes
   before publishing the complete view.

The upper limits are 524288 parents/nodes, INT_MAX pair capacity and 2 GiB for
each complete host/device payload cap. At the parent limit the possible complete
pair count still fits uint64; a count above pair capacity fails without fill.
The complete SAP count remains quadratic in the worst case; this slice does not
claim a full-vehicle runtime bound or accelerate pathological all-overlap inputs.
A key encodes `(min_parent << 32) | max_parent`, in strictly increasing order,
independent of axis/tie scheduling. A successful empty set is
`{nullptr,0,complete=true}`. Shared-node, same-body, removed or otherwise excluded
parents remain in the inventory until an external authenticated policy filters
specific features. No nearest-K limit, floating atomics or pair truncation exists.

Input/bounds/capacity failure revokes the current view and permits retry. CUDA
failure additionally poisons storage. Previously saved raw pair pointers expire
at the next Evaluate or destruction; they are not owner/attempt receipts.
Consumers must check the current report and complete view. The adapter does not
publish a success while its work is merely enqueued.

## Full payload forecast

One device allocation owns typed compact parents, two AABB arrays, two axis-key
arrays, two sort-index arrays, uint64 count/offset arrays with sentinel, two
pair-key arrays, control, alignment padding and one 256-aligned CUB temporary
region sized to the maximum of the three actual sort/scan queries. These phases
share scratch only after ordered completion. No Evaluate allocation occurs.

`device_bytes` includes all of that arena. `owned_host_bytes` includes the adapter,
implementation/layout, persistent scalar readback and complete retained S0
backing; `retained_source_bytes` exposes its sharing contribution for composition.
`startup_host_bytes` also includes compact upload plus represented query/layout
staging (`QueryStagingBytes`). These are payload bounds, not driver/allocator RSS
claims. Borrowed caller coordinates and consumer pair-readback storage belong to
the caller's forecast. Existing HydroelasticBroadphase allocation, stream, filter,
cap and failure behavior remains unchanged by default.

## Evidence and gate

The complete 30028be `.cu`, `.cuh`, and `Func.cuh` donors are retained in reference/.
The verifier proves the complete owning `.cu` unchanged, exact AABB extraction,
unchanged bounds/sort/reorder kernels, selected sweep/filter/write extraction,
and wrapper order. Existing seven CUDA broadphase functions remain an affected
regression gate. The only old receipt refresh is collision/BUILD in
mapped_wall_assembly_inputs, preserving its old and exact adapter-base records;
its prior S0/S1 additions and this additive packaging are explained there.

Author limits: one CPU, 512 MiB. Four host functions pass, including 513 actual
immutable typed source layers, independent exhaustive all-axis sets, unchanged
legacy policies, complete aligned forecast and exact-cap retry. C++ and three
CUDA-shaped units pass host syntax (CUB query declarations mocked). This is not
NVCC, CUDA execution or CUB ABI qualification.

Five authored actual CUDA functions cover mixed T3/Q4 strided inputs, all axes,
explicit crossing endpoint sweep, more than one block/full coincident inventory,
invalid/capacity/alias/lowest-source-error and retry, exact caps/source lifetime,
complete zero sets, and a recoverable invalid-launch error revoking/poisoning a
prior success. The independent test AABBs use exact long-double sums for the
bounded represented test coordinates; the pair oracle is exhaustive and does
not invoke the production sweep or inflation.

Root executes the following under the shared workstation guard, after the active
preview. No full-model configuration or native donor build is required:

```sh
cmake -S lib_utest/qualification/self_contact_broadphase \
  -B ../crash-work/cache/self-contact-broadphase-root-1 \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_CUDA_ARCHITECTURES=120 \
  -DSELF_CONTACT_BROADPHASE_CUDA=ON
cmake --build ../crash-work/cache/self-contact-broadphase-root-1 --target \
  self_contact_broadphase_host self_contact_broadphase_cuda self_contact_broadphase_legacy -j1
ctest --test-dir ../crash-work/cache/self-contact-broadphase-root-1 --output-on-failure
python3 -B lib_utest/qualification/self_contact_broadphase/verify_sources.py
```

Owning targets: `//lib_src/collision:self_contact_broadphase`,
`//lib_src/collision:hydroelastic_broadphase`,
`//lib_utest/qualification/self_contact_broadphase:host` and `:cuda`.
The CTest names are `self_contact_broadphase_{host,cuda,legacy,identity}`.
