# Mapped wall node-status scheduling

The active mapped wall caller performs two point evaluations and one two-phase
scatter per interval. Each already has independent CUDA node workers, followed
by a single-thread scan of every compact node's status. This change replaces
those four scans with integer `atomicMin` on the compact index inside the existing
workers. It copies the winning complete `Control` after the same stream boundary.
The first global NID is not the error-order key.

The baseline is TL `92e8cc47ef834daae4e73cfba577c575db0fc740`. The latest root
two-step full V5 measurement reported wall assembly 0.419644 s and evaluation
0.252055 s per interval, about 43% of 1.5635 s preparation. These are inclusive
wall-stage timings, not measurements of the four scans. No speedup is claimed
before root CUDA and full-model timing.

`BeginPoints` and `BeginScatter` reset the existing `Summary::parent_failure`
word on the supplied stream. Point completion resets it before parent work;
stiffness completion resets it before force work. Earlier assembly/response
failures keep their original control and bypass later consumers. Point admission
continues to use `points_admitted`, including the old behavior where an admitted
point failure replaces an earlier control. Scatter without a summary retains
the complete old source-order scan. A malformed index or an indexed successful
slot also falls back to that scan. The private summary comes only from the
retained mapped allocation; arbitrary concurrent external modification is not
an admitted interface.

Point, share, parent, force, STI and observer arithmetic is unchanged. There is
no floating-point atomic, reordered sum, new allocation, host copy or host
synchronization. All six force/couple arrays and both stiffness arrays remain
unpublished until both scatter phases pass. `Layout.h` changes only the lifetime
comment. Host and device forecast increments are both **zero bytes**.

`FrozenEvaluation.cuh` and `FrozenScatter.cuh` retain the complete baseline
callers. Only include paths, namespace and qualification of the unchanged
observer helper differ; the source verifier reconstructs and authenticates the
exact original bytes. It independently checks all floating-point producer
bodies after removing only the integer failure-selection call. The old point,
parent, serial observer and serial scatter leaves remain separately pinned.
Four earlier manifests retain their original manifest hashes and every replaced
row with the narrow scheduling reason; nested verifier hashes are updated in
dependency order. No frozen reference is changed.

Author checks under one CPU/512 MiB: four host functions pass; five CUDA-shaped
C++ syntax units pass, including the active `Operations.cu` and existing
evaluation/scatter tests; the new source gate and nested response/input/interval/
observer/scatter identities pass. Syntax checks do not compile or execute CUDA.
Reports are `crash-work/reports/mapped-wall-node-status-author-{tests-1,syntax-2,identity-2}.json`.

The three new CUDA functions compare the complete old and new callers on an
explicit nonblocking stream: mixed active masks, both observer modes, signed
zero/cancellation, base reset, multi-block first/multiple/last faults, parent
failure, earlier admission failure, stiffness-before-force priority, null-summary
legacy mode, unchanged eight destination arrays, stale summary reset and retry.
Physical point/parent/diagnostic results use the existing exact comparison; no
tolerance is added.

Root gate from the integrated TL tree, under the normal workstation guard:

```sh
cmake -S lib_utest/qualification/mapped_wall_node_status -B <new-build> -DMAPPED_WALL_NODE_STATUS_CUDA=ON -DCMAKE_BUILD_TYPE=Release -DCMAKE_CUDA_ARCHITECTURES=120
cmake --build <new-build> --parallel 1 --target mapped_wall_node_status_host mapped_wall_node_status_cuda
ctest --test-dir <new-build> --output-on-failure
python3 -B lib_utest/qualification/mapped_wall_node_status/verify_sources.py
```

Expected new numerical count: **4 host + 3 CUDA**. The verifier also runs the
existing response, assembly-input, interval, global-observer and serial-scatter
source entrypoints. Rebuild/run the existing `mapped_wall_evaluation_check`,
`mapped_wall_scatter_cuda`, `mapped_wall_response_cuda`,
`mapped_wall_response_owner`, and `physical_mesh_wall_cuda` targets in their
existing caches. The actual-owner gate retains the complete rollback/retry and
accepted-history boundary. Owning Bazel targets are
`//lib_src/collision:nodal_wall_mapped` and
`//lib_utest/qualification/mapped_wall_node_status:host`.

Finally rerun the same full V5 loaded/retry archive workload and compare exact
saved force, stiffness, work, activity, screening and archive observations.
Record inclusive wall-stage time and unchanged memory forecast. Root owns all
native/CUDA execution and original-source runs.
