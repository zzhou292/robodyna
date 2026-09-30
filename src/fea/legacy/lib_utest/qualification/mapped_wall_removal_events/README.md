# Mapped wall ordered removal events

This changes only the integer scheduling inside the existing candidate finish.
Every parent receives the same fresh accepted/proposed byte checks on every
call. A single 256-thread block overwrites eight invalid/removal bitmap pairs
per tile; lane zero visits their union in increasing source-parent order.
There is no count-, model-, activity-history- or no-removal special mode.

The qualified `Sum` and complete old `RemovedPotential` remain unchanged.
The original certified addition executes for every valid removal, including
zero and negative zero. Potential values are read only when lane zero reaches
that event. Nonremoved and invalid-parent payloads remain unconsumed; the
potential's `error` field is still unconsumed by `Sum`. No certificate is
prefetched or prevalidated. Earlier cumulative arithmetic failure beats a
later invalid mask. The previous summary prefix and all control/diagnostic
fields match the old caller on failure. Prior control failure prevents summary
reset and all activity/potential reads. Only complete success publishes valid
and the retained current response rate.

Both barriers are uniform, including the partial final tile and a failed
ordered fold. Each warp writes both masks even when empty. The public admitted
parent limit proves that the unsigned tile increment cannot wrap. Activity
arrays, parent records and storage retain the existing immutable/private
within-call lifetime contract; later speculative **byte classification** can
occur, but it cannot publish an error or consume a potential before its turn.

The new POD shared tile is exactly 64 bytes. There is no new global allocation,
arena/header/host forecast field, kernel launch, host transfer, owner, source
admission or public API. Valid-call logical reads are two bytes per parent plus
the three consumed doubles per removal; physical memory transactions remain
hardware dependent. Dense removal retains all serial additions, so speedup is
not assumed for that workload.

## Evidence and qualification

The source authority is full `edab8e1` Operations, Kernels and Reduction files
retained under `reference/`. `Frozen.h` keeps complete old removal and finish
bodies with only namespace/linkage adaptation. `removal_proof.py` reverses the
three exact operation substitutions and verifies the complete old file SHA.
The existing node-status -> response -> assembly-input -> interval/observer
and scatter receipt chain consumes that checked inverse and preserves its
previous complete-file/body checks and manifest histories.

Five Release host functions cover all 65536 byte pairs, all admitted-cap and
warp/tile boundaries, dense/sparse/all-retained/all-inactive patterns, ignored
NaNs, signed zero, subnormal values, finite overflow, invalid certificates,
competing errors across tiles, partial output and same-allocation retry.
The authored CUDA tests compare the complete old finish caller, including
prior-error behavior, on non-default streams. Entire control, summary and
diagnostic packets compare bitwise; source buffers are checked unchanged.
CUDA events report eight repeated complete calls at 131072 parents for
retained, sparse-removal and dense-removal patterns. No speed threshold is
invented. Root executes CUDA and the existing actual-owner removal gates.

Historical actual V5 kernel32 measured FinishCandidate at 28.944039 ms/call;
the root's subsequent qualified profile reported 28.821 ms/call. These are
baseline observations, not a measured benefit from this implementation.

## Root commands

Run under the owning resource guard:

```sh
cmake -S Total-Lagrangian-FEA/lib_utest/qualification/mapped_wall_removal_events \
  -B crash-work/build/wall-removal-root-1 -DCMAKE_BUILD_TYPE=Release \
  -DMAPPED_WALL_REMOVAL_CUDA=ON -DCMAKE_CUDA_ARCHITECTURES=120 \
  -DCMAKE_CUDA_FLAGS=--ptxas-options=-v
cmake --build crash-work/build/wall-removal-root-1 --parallel 4
ctest --test-dir crash-work/build/wall-removal-root-1 --output-on-failure
crash-work/build/wall-removal-root-1/mapped_wall_removal_cuda \
  --gtest_filter=WallRemovalCuda.SparseAndDenseSyntheticTimingWithExactPackets
```

Rebuild/run `physical_mesh_wall` with `PHYSICAL_MESH_WALL_CUDA=ON`, including
`ActualT3RemovalUsesBaseMaskThenPostRemovalZeroAndCaptureRetry`, late scatter
failures, owner/alias checks and allocation caps. Rebuild affected mapped-wall
node-status, response, assembly-input, interval, observers, scatter and
evaluation owning gates. Then compare full V5 all 54 archive files, non-timing
native JSON and exact forecast to the prechange baseline, and reprofile the
active finalization kernel. Actual GPU qualification/performance is pending
the root gates; author checks are host/source/C++ shape only.
