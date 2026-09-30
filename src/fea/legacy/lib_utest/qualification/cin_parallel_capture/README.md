# CIN final acceleration capture

This slice moves only the final non-rigid-node acceleration copy out of
`CompleteCin`. The previous complete caller at `d11b67c` is retained in
`reference/ExplicitNodalCinStep.cu` and independently compiled by `Frozen.cu`.
The public owner still uses its original attempt, selector, capture allocation,
stream and readback authority.

`CompleteCin` preserves its exact ordinary failure-key check, ordered rigid
candidates and member orientation, CIN recovery, and ordered secondary drift
and orientation checks. Only after that kernel completes does `Capture.cu`
copy the six doubles for each non-rigid node. Any nonzero rigid-member mask
skips that row; the ordered rigid stage already owns its capture. Secondary
capture therefore observes recovered acceleration rather than pre-recovery
work. Copies retain every double's bits; there is no arithmetic or reduction.

The kernel reads status before any row, work or capture output. Failed suffixes
keep the old partial rigid capture and leave all ordinary/secondary capture
untouched. Disabled capture returns on the host before kernel launch and reads
no control or work. The admitted sink retains separate node/node-rotation
slabs; each lane has unique six-value destinations. The same stream orders the
suffix before these writes and existing owner readback/publication after them.
Kernel launch/runtime failures still follow the existing rejected-attempt
path. No accepted state is written here.

The grid uses 128 threads and at most 256 blocks, with a stride over the exact
admitted domain. Existing owner maximum is 524,288 nodes, so all index and
stride operations remain within their existing integer range. No Input field,
host/device tail, allocation, header or startup forecast changes. Zero extra
host bytes and zero extra device bytes are required.

Three host functions compare the complete old copy with node iteration across
block/tail boundaries, nonzero and absent rigid masks, signed zero, subnormals,
infinities and a NaN payload. These nonfinite raw packets test copy bits only;
they provide no new mechanics admission. Four new CUDA functions cover the
same values, disabled/failed capture with unconsumed invalid pointers and a
same-allocation retry, and the complete frozen caller over three half/full-kick
intervals with repeated triangle slots, rigid groups, capture on/off, all prior
fault priorities and late rigid/secondary orientation faults. The existing
actual PART/plain+CIN owner capture/discard/retry test is reused unchanged.
Every state/work/load/capture double is compared bitwise for complete callers.

`verify_sources.py` proves the complete caller has only the explicit copy
removal and launch addition. It checks the helper against the old loop, keeps
all frozen source bytes, and invokes the preceding screen/force/ordinary source
proofs. Their former receipt hashes and changed records are retained in their
manifests; they now recognize this exact final-copy extraction. Current Input,
layout, storage and capture-layout files are authenticated unchanged.

Root owns CUDA/native/full-vehicle execution:

```sh
cmake -S lib_utest/qualification/cin_parallel_capture -B <new-build> -DCIN_PARALLEL_CAPTURE_CUDA=ON -DCMAKE_BUILD_TYPE=Release -DCMAKE_CUDA_ARCHITECTURES=120
cmake --build <new-build> --parallel 1
ctest --test-dir <new-build> --output-on-failure
```

Expected: 3 host functions, 4 new CUDA functions, 1 reused owner CUDA function,
and source identity. Bazel targets are `:host`, `:cuda`, and the existing
`//lib_src/solvers:explicit_nodal_state`. Shared fixture visibility is additive;
no owner fixture is copied.

Affected root regressions: CIN parallel screen, force inputs and ordinary
stages; physical timestep/main summary; rigid assembly owner; tied CIN/native
runtime and common physical publisher. The identical loaded vehicle gate can
compare all accepted observations and inclusive CIN timing. The preceding
1.483132 s/attempt is an inclusive observation, not measured copy-kernel cost.
