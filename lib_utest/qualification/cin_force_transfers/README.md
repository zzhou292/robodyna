# Prepared CIN force rows

This slice parallelizes only the pure row preparation inside the actual CIN
advance. The public serial `PrepareForceTrial` remains a complete sequential
route. No force law, timestep, source policy, accepted state or clock changes.

The active call chain is `AdvanceStaggeredCin` -> `AdvanceSealedNodal` ->
`LaunchCinAdvance` -> `cin_advance::Launch`. Existing parallel input checking and
entry-IN copy finish first. A new kernel prepares current patch cofactors,
transferred loads and coefficient increments independently for each row. The
existing one-thread prefix then applies those packets in original row order,
including all four original slots for a repeated T3 master. Ordinary/rigid
screening, motion, CIN recovery, drift and capture retain their current order.

Startup in `NodalCinStartup.cpp` first proves unique secondary nodes, builds the
complete dependent mask, then rejects every master in that mask. It also rejects
CIN/rigid overlap. A row leaf consumes accepted positions, its own secondary
force/couple/M/J/STI/STIR and the completed entry-IN snapshot. Earlier application
can change shared masters but cannot change any later leaf input. No result is
cached across attempts. A malformed raw private topology is not a new admitted
source profile; the actual owner always supplies its authenticated source.

`CinForceTransfer.h` keeps the old expressions and consumed values. Application
still publishes patches and checks numerical mass and then each master at the
original locations. A prior application overflow wins over a later prepared
patch/transfer error. Failed complete input validation prevents every leaf read
and packet write. Control also gates the ordered apply, so stale packets from a
discarded attempt are never consumed; a successful retry rewrites every packet.

## Storage

The separate startup-owned tail is 512 bytes per row on the qualified host ABI.
It is not a history/cache selector and needs no host seed. Existing device-arena
prefix offsets are unchanged. `sizeof(CinStorage)` grows from 536 to 568 bytes;
the added region metadata and device pointer add 32 host bytes. No per-attempt
allocation or new full-node array is introduced. Exact cap checks precede owner
allocation and no cap is increased.

For the actual V5 counts (376,930 nodes, 11,165 rows, 13,173 witnesses, 779 groups):

| CIN allocation | Baseline bytes | Prepared-row bytes |
| --- | ---: | ---: |
| Optional host backing | 1,375,436 | 1,375,468 |
| Device arena | 30,239,504 | 35,955,984 |
| Device arena plus both coefficient tails | 54,720,320 | 60,436,800 |

The device delta is exactly 5,716,480 bytes. These are CIN-owned amounts, not a
replacement for the complete owner's inclusive forecast. At the explicit maximum
65,536 rows, this tail alone is 33,554,432 bytes; complete layouts must still fit
the existing host/device caps.

## Qualification

`reference/` preserves complete raw force-header and CUDA-caller bytes from
92e8cc4. `FrozenForce.h` and `Frozen.cu` adapt only includes/namespaces and redirect
the force call to that complete frozen header. Existing already-qualified input,
screen, group and capture kernels remain shared. `transfer_proof.py` proves the
exact leaf/application extraction and reverses only the launch changes before
the retained older CIN source proofs run. Earlier receipt values and all frozen
donors remain recorded.

Five host functions compare named double bit patterns through three force phases,
shared masters, repeated slots and 1/2/127/128/129-row boundaries; distinguish
earlier apply failures from later leaf failures; check untouched input-failure
fields, reverse leaf order/input independence, actual layout and exact caps.
The CUDA target has three full-caller packet functions plus the retained ordinary
owner failure test and a new actual-owner late-transfer failure/complete retry.
The latter keeps all accepted fields, rigid snapshots, capture and allocation
counts across epoch-zero and ordinary intervals.

Root-only owning CUDA gate (run inside the usual root resource guard):

```sh
cmake -S "$TL/lib_utest/qualification/cin_force_transfers" \
  -B "$BUILD" -DCMAKE_BUILD_TYPE=Release -DCMAKE_CUDA_ARCHITECTURES=120 \
  -DCIN_FORCE_TRANSFERS_CUDA=ON
cmake --build "$BUILD" -j2
ctest --test-dir "$BUILD" --output-on-failure
```

Owning Bazel targets are `//lib_src/solvers:explicit_nodal_state` and
`//lib_utest/qualification/cin_force_transfers:{host,cuda}`. Re-run affected input,
ordinary, screen, groups, capture, native tied-CIN, physical timestep/main-summary
and actual rigid/CIN owner gates. The full V5 same-input loaded prefix must retain
the complete archive hashes and inclusive allocation forecast. Report actual
`advance_cin` timing; no isolated row-stage timing or speedup is claimed from
source inspection or host execution.

Author checks are bounded host execution, C++-shaped syntax and source identity.
They do not establish native/CUDA arithmetic or performance qualification.
