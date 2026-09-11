# Accepted parent activity

The additive byte readback is 0 for inactive and 1 for active. It reads the
complete validated accepted native history, including at epoch zero only after
real source/virgin-history binding. Prepared history never supplies these bytes.
QEPH/T3 require the explicit failure-capable scope; QBAT keeps its distinct
parent mask. No activity is inferred from a material declaration.

The common publisher validates actual participant object identities, the exact
complete inventory and the live owner's accepted endpoint. This is a read-only
composition check, with no new selector, allocation, clock or mechanics.

Configure this directory with the normal owning CUDA toolchain; build and run
`shell_parent_activity_cuda_test`. The fixture reuses the qualified complete
Q/T/B resident owner and actual removal trajectory. It checks virgin binding,
prepared/accepted separation, foreign scope, stale endpoint, complete capacity,
late device-read corruption and discarded-attempt retry. These tests qualify the
readback contract; they do not create a full vehicle/CIN source owner.

## Prepared activity for precommit contact assessment

The separate `CopyPreparedParentActivity(owner, token, expected, bytes, count)`
entry reads a complete candidate, preserving the accepted API above. The exact
pending family diagnostics and actual owner's borrowed prepared view must match.
Exact capacities and all input/retained-state aliases reject before readback.
QEPH and T3 reuse complete failure-section staging; T3 additionally validates its
true one-point history against the actual shell slab. QBAT reuses its complete
four-point result validation. Only after all parents validate are bytes copied.

A failed typed read invalidates that family's candidate; a second read on the
same attempted candidate rejects. Accepted histories/flags remain unchanged.
The caller discards the common attempted step and retries with a fresh token.
There is no new slab, history allocation, activity default, clock or publication.

Three new CUDA functions check all three families against their typed candidate
history, actual owner/token and aliases, true T3 one-point/QBAT removal over the
existing 24-interval trajectory, and last-parent read corruption followed by
same-attempt rejection and fresh retry. Four existing accepted/CIN activity
functions remain in the owning target. The new dependency has only host syntax
and source-identity author checks; actual CUDA execution belongs to root.

Author evidence: `shell-prepared-activity-author-2` compiles the three owning
readback units and the new CUDA test's host-callable code under one CPU /512 MiB.
Attempt1 retained the missing Eigen include-path error; attempt2 adds the owning
Eigen include path without changing production. `shell-prepared-activity-identity-1`
checks the live T3 startup/force and QEPH provenance (56/67/76 records). Only T3
build registration and chained live pins changed; previous hashes remain in the
manifests, and native donor/arithmetic bytes are unchanged.

Root gate, using a fresh bounded build and evidence prefix:

```bash
cmake -S <TL>/lib_utest/qualification/shell_parent_activity -B <fresh-build> \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_CUDA_ARCHITECTURES=120
cmake --build <fresh-build> --parallel 1
ctest --test-dir <fresh-build> --output-on-failure
```

`shell_parent_activity_cuda_test` contains seven functions: four unchanged
accepted/CIN and three prepared-activity tests. The owning Bazel libraries are
`//lib_src/elements/qeph:batch`, `//lib_src/elements/t3:batch` and
`//lib_src/elements/qbat:batch`; each depends on the small header-only
`//lib_src/elements:shell_prepared_activity_readback` preflight target.
