# Fixed self-contact runtime transaction

`SelfContactTransaction` privately owns one
`SelfContactForceAssembly` and exactly one
`ShellPhysicalScratchParticipation`. Startup reauthenticates the retained
active-use physical binding, actual publication owner/participants, startup,
configuration and qualification IDs. The caller can obtain only the immutable
SelfContact roster entry.

Accepted assembly first runs the existing deterministic VF force/STI
implementation, then records that same owner/token/view on the private issuer.
Every later failure discards owner, common publication, force and issuer trial
scratch. Caller output changes only after both operations succeed.

Candidate sealing is not a pass-through. The transaction copies actual
accepted and prepared positions from `FENodalState` into fixed startup storage,
runs current regularity over every active parent, rebuilds exact fixed facets,
and reruns complete discovery and represented interval crossing for the
declared bounded roster. Unresolved results fail. Nonlocal crossings require an
exact policy decision; rejection aborts the trial, while admission requires
the crossing's canonical VF identity to have been present in the accepted
force batch. Local fixed-facet exclusions remain explicit.

The final nonaggregate `SelfContactTransactionReceipt` is tied to the exact
owner/base/attempt/source/configuration/qualification/active-use identities.
Only that receipt can expose the private typed physical receipt in a
`ShellPhysicalScratchReceiptRoster`.

This safe first slice requires every selected self-contact parent to remain
active. The missing API for removals is a physical-publication-authenticated,
active-use-ordered parent activity receipt; caller byte arrays cannot safely
authorize skipped geometry. Broadphase/device-host candidate roster production
also remains outside this module.

## Author host gate

Run with one CPU and a 512 MiB cap under
`crash-work/reports/connection-author.lock`:

```sh
cmake -S lib_utest/qualification/self_contact_transaction \
  -B <host-build> -DCMAKE_BUILD_TYPE=Release
cmake --build <host-build> --parallel 1
ctest --test-dir <host-build> --output-on-failure
```

This gate runs value, exact-cap, crossing-policy, source and C++ syntax tests.
It invokes no NVCC or GPU.

## Root CUDA gate

The parent CUDA workstation owns:

```sh
cmake -S lib_utest/qualification/self_contact_transaction \
  -B <cuda-build> -DCMAKE_BUILD_TYPE=Release \
  -DSELF_CONTACT_TRANSACTION_CUDA=ON \
  -DCMAKE_CUDA_ARCHITECTURES=<root-arch>
cmake --build <cuda-build> --parallel 1
ctest --test-dir <cuda-build> --output-on-failure
```

That target uses real `FENodalState`, physical publication, current regularity,
fixed discovery, represented crossing, VF force and CIN STI. It covers missing
mandatory receipt rollback/retry, initial half-kick, ordinary interval and
allocation stability. CUDA execution is intentionally left to the parent.
