# Mapped wall response scheduling

The diagnostic-only original V5 run measured mean `CheckResponse` 0.558229 s, compared with 0.168955 s for Scatter and 0.1036 s for Evaluate. All archive and viewer hashes matched the baseline. Evidence is `crash-work/reports/wall-stage-profile-analysis-1.json` and `wall-stage-profile-run-1.log`. That temporary instrumentation is not part of this change. The production improvement is not timed by the author.

This slice replaces only the mapped accepted-assembly response launch. It preserves the complete old `Response` and `CheckResponse` bodies, activity, point evaluation, scatter, local step arithmetic, candidate observations and legacy contact entrypoints. No public diagnostic, mechanical coefficient, force law, timestep, clock or participant is added.

`ResponseIncidence.h` builds group-to-compact-row CSR once from the authenticated, immutable root map. Each group's rows remain in ascending compact contact row order, which need not match global node IDs. The builder validates every borrowed root before writing and uses its owned offsets as construction cursors before restoring them. It retains no borrowed source pointer or extra staging allocation. Fresh disjoint arena regions are an internal caller contract; this is not an arbitrary mutable CSR interface.

`Response.cuh` schedules four stream stages:

1. On an otherwise successful attempt, reset the existing integer failure word.
2. Independent ordinary rows compute the original `UpperProduct`. A 128-thread block reduces their finite nonnegative maxima. A zero upper stiffness consumes no root, inverse, coordinate or body fields.
3. One worker per rigid group calls the unchanged `EvaluateRigidNormalResponse` and `AccumulateRigidContactTrace` in original compact row order. Empty groups produce the original positive zero.
4. If any worker reported an error, execute the complete original serial Response. It resets the scratch rate/traces and recreates the original first-error node, partial trace/rate prefix and untouched diagnostic rate. Otherwise combine the block maxima and ordered group traces. Then execute the exact old square-root, outward multiplication and `step >= 1.6` test.

Workers never modify control, result diagnostics, forces or STI. Only integer `atomicMin` arbitrates fallback; it does not choose a new error policy. Successful ordinary values are finite and nonnegative, so `fmax` association cannot change their bits. Rigid additions are not reassociated. The old serial fallback owns every numerical-error/partial-output result. Private incidence is constructed from admitted immutable roots; corrupt sparse storage is not a newly admitted source profile. An earlier stage failure bypasses every response read/write. The integer word is reset before later users, and unchanged scatter control gating prevents any partial owner contribution on response failure.

The new sidecar tail is `4*(nodes + groups + 1) + 8*ceil(nodes/128)` bytes plus alignment, with at most 4096 block maxima and the existing 524288-node/1024-group caps. At 359785 surface nodes and 779 rigid groups it adds **1464752 B to each host/device arena**. `Scratch` is 40 B and the three layout regions are 72 B on the qualified author ABI; the two bound sidecars and layout are already included by `sizeof(Impl)` in the inclusive forecast. No new device allocation or host staging container is needed. Output alias protection already covers the complete host arena. Exact-cap and one-byte-short tests exercise the actual layout, and the actual owner test checks the complete public device forecast.

## Qualification

The six host functions cover ordered interleaved rigid groups, ordinary maxima, sparse reversed global IDs, block boundaries, empty/all available groups, exact step neighbors, zero/negative-zero and negative duration legacy behavior, zero-stiffness consumed-only inputs, seeded prior rejection, invalid root/mass/position, ordinary underflow/overflow, rigid accumulation overflow, multiple competing faults, exact partial diagnostics and retry. CSR tests independently construct each group's expected row list. The complete baseline files are frozen under `reference/`; `FrozenResponse.h` changes only include/namespace/host-device annotation. The source gate proves complete fallback/step identity and the one-launch caller change, and delegates all prior observer/interval/scatter/source controls. Earlier manifests and changed rows are preserved in the follow-on receipts.

Three CUDA packet functions run actual kernels against the complete frozen CUDA response, including same-allocation retry and earlier-error gating. The actual-owner function injects a finite positive inverse only after source admission to trigger response overflow or StepTooLarge, verifies all eight force/couple/STI arrays and accepted histories remain bit-identical, restores the deliberate fault and retries. Existing physical wall CUDA tests retain rigid/CIN loaded commit, late scatter rejection, removal-mask and capture rollback coverage. No tolerance is introduced.

Author result: six host functions, three production C++ syntax units, three CUDA-shaped C++ syntax units, source identity and owning CMake configuration pass under one CPU/512 MiB. The shape checks do not compile or run CUDA. Evidence: `crash-work/reports/wall-response-author-4.json` and `wall-response-author-functions-4/host.xml`. Failed author iterations 1 and 2 are retained; both were test-fixture issues. Independent read-only review found no production blocker in the admitted immutable-root profile.

Root commands, under the workstation guard, from the integrated TL tree:

```sh
cmake -S lib_utest/qualification/mapped_wall_response -B <fresh-build> -DMAPPED_WALL_RESPONSE_CUDA=ON -DMAPPED_WALL_RESPONSE_OWNER=ON -DCMAKE_BUILD_TYPE=Release -DCMAKE_CUDA_ARCHITECTURES=120
cmake --build <fresh-build> --parallel 1 --target mapped_wall_response_host mapped_wall_response_cuda mapped_wall_response_owner physical_mesh_wall_values physical_mesh_wall_cuda
ctest --test-dir <fresh-build> --output-on-failure -R '^(mapped_wall_response_|physical_mesh_wall_)'
```

New expected count: **6 host + 3 CUDA packets + 1 actual-owner CUDA function + source identity**. Existing physical wall contributes 5 host + 3 CUDA functions. Owning Bazel targets: `//lib_utest/qualification/mapped_wall_response:host`, `:cuda`, and production `//lib_src/collision:nodal_wall_mapped`. The reused complete owner fixture remains CMake-owned.

Run affected mapped assembly-input/evaluation/observer/interval/scatter, physical publication and legacy wall regressions. The final original V5 loaded archive should retain the baseline exact archive/viewer hashes, force/penetration/potential/work/activity and screen results. Compare inclusive wall-assembly timing; do not label it an isolated response-kernel measurement. The host/device forecasts must include the new tail.
