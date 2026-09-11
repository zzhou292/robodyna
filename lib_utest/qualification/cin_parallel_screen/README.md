# Parallel ordinary-node CIN structural screen

This scheduling change consumes the same actual post-transfer M/J/STIFN/STIFR and source role maps as the existing CIN screen. It does not change a force, coefficient, native scalar timestep, rigid trace, motion, recovery, capture or publication operation. The rigid trace remains the existing analytical surrogate; this is not a new native RBYM formula claim.

`cin_timestep/{Sources,ScreenValues,Screen}.h` separates the exact existing header checks, one node, and ordered group suffix. The public serial value API still visits every node in domain order. `cin_advance/{ScreenSummary,Screen}.h` and `Screen.cu` provide the private owner path. The force input scan/copy and ordered force transfer finish first. A screen header kernel validates the original header, 128-lane node blocks evaluate independent rows, and one bounded reduction selects the result before executing the unchanged ordered rigid group/member suffix. Ordinary motion and the entire rigid/recovery/drift/capture suffix follow only after admission. Disabled screening and a null private screen pointer retain the old serial branch.

## Exact selection and failure ordering

Each node consumes all four coefficients before any dependent or rigid role skip. Translation, rotation-presence and fixed-rotation branches keep their original order and raw mask interpretation. `OrdinaryLimit` is unchanged. Each summary contains only an already rounded finite positive minimum, its domain row, and the first invalid row. No floating value is added or atomically accumulated. Merging chooses the lexicographically smallest `(dt, domain row)` and the minimum invalid row; these selections are associative on the validated values. The `DBL_MAX/UINT32_MAX` empty record preserves the strict old sentinel, including a computed bound equal to `DBL_MAX`.

Any ordinary error is reported before the first rigid group is consumed. On an otherwise successful ordinary scan the suffix begins with `invalid_node = node_count - 1`, preserving the old malformed-first-group result. Subsequent malformed ranges retain the last visited earlier group member. Strict unchanged `Include` preserves an ordinary winner on an equal group limit, and preserves the first group on equal group limits. The index is the domain row, not source NID.

Header/node/group errors leave the previous owner limit and ordinary motion key unchanged. A valid too-large step publishes the complete old computed minimum and limiting node, while leaving that key unchanged. Only admitted screening initializes the ordinary key. Earlier failed force stages skip all screen reads and writes. The reduction is scratch-only; accepted state, motion and capture arrays are not written.

## Storage and actual owner route

A trivial 16-byte summary has one double and two uint32 indices. At most 256 block summaries require 4,096 device bytes, independently owned after the two existing 8-byte error keys. No host scratch or host initializer is added. The `CinStorage` pointer plus `CinLayout` region add 32 bytes on the owning ABI, charged through the existing actual-size host forecast. Every node kernel fully initializes its record before completion reads it. CUDA shared storage is 2,048 bytes per block. Allocation counts do not grow per attempt.

`CinStorage::Upload` binds the counted device arena region and the actual owner passes it into its private launch input. The first private record receives the completed ordinary summary for packet qualification; no route flag or other state was added to public diagnostics. Layout exact-cap/late-host-and-device failure/retry tests, source identity assertions, and the existing public-owner rejection/retry tests cover this connection. The original 372,435-node / 11,165-row / 13,173-witness layout takes the full 4,096-byte tail.

## Independent qualification

`reference/Screen.h` and `reference/ExplicitNodalCinStep.cu` retain complete baseline `8f8fb0ae49190eabf383ebd081198a2c3dde7992` files. `FrozenScreen.h` changes only include paths and namespace; scalar and rigid math dependencies are independently pinned and unchanged. `FrozenCaller.inc` retains the full old preparation, ordinary kernel and rigid/recovery/capture suffix, and `Frozen.cu` retains the complete old launch sequence. It does not call the new Screen helpers. The existing complete force-input and ordinary-stage frozen receipts remain part of the source gate.

Six new host functions cover tree grain/maximum-block boundaries, exact scalar extremes, coefficient-before-role and header priority, sentinel/equal minima, malformed first/later groups, consumed-only values and exact caps/retry. Five new CUDA functions compare full frozen CUDA Screen and full frozen caller, including half/full kicks, three intervals, both capture settings, rigid groups, role masks, coefficient/group/step failures, key and accepted/motion/capture preservation, and retry. The CUDA target also runs the two unchanged `cin_physical_timestep` public owner functions, including transferred CIN stiffness, ordinary and PART/plain limits and accepted-state rollback. There are no relaxed numerical tolerances.

The author performs only bounded source identity and C++/CUDA-body syntax checks. Numerical host/native/NVCC/CUDA/full-model execution belongs to the root qualification lane.

Root owning CMake commands (inside the workstation guard, with its selected CUDA architecture):

```sh
cmake -S lib_utest/qualification/cin_parallel_screen -B <build> -DCIN_PARALLEL_SCREEN_CUDA=ON -DCMAKE_BUILD_TYPE=Release -DCMAKE_CUDA_ARCHITECTURES=120
cmake --build <build> --parallel 1
ctest --test-dir <build> --output-on-failure
```

Expected numerical functions: 6 host plus 5 new CUDA plus 2 retained actual-owner CUDA; one source identity CTest. Owning Bazel targets are `//lib_utest/qualification/cin_parallel_screen:host` and `:cuda` plus `//lib_src/solvers:explicit_nodal_state`.

Affected root gates: `cin_force_inputs`, `cin_parallel_ordinary`, `cin_physical_timestep`, `cin_physical_main_summary`, actual tied CIN/native runtime, rigid assembly owner, physical publication and the complete loaded wall case. Measure the existing inclusive CIN stage on that case; this change adds no profiler or timing synchronization and makes no claim of isolated kernel time.
