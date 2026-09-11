# Complete resident collections with optional constant failure

`ShellBatchFailureBinding` is an immutable, complete source-order declaration
alongside the existing explicitly heterogeneous section catalog. Every parent
has a policy: `None` or LAW44 `ConstantAllPoints` with positive native D1.
`Tab1AnyPoint` is reserved and rejected. LAW1 remains non-failing. An all-None
collection uses the existing initializer.

The new `InitializeJoined(config, binding, catalog, failure, limits)` overload
allocates one optional failure arena per complete native family. The existing
mixed arena, saved material history, force slabs and publication selector stay
in place. Point damage/masks/time, current unmasked force stress and completed
parent activity occupy two family-parent-indexed sidecar slabs. There is no
additional selector or clock. Both contributors must retain the same complete
catalog **and** failure declaration, including the other family's rows.

`CopyAcceptedFailureHistory` and `CopyPreparedFailureHistory` require an exact
family count and complete accepted/prepared identity. They validate both the
saved typed material section and failure sidecar before publishing caller
output. Parent activity is an actual native bool, independent of point masks
and contact eligibility. A numerical candidate/readback failure leaves the
accepted selector unchanged; a CUDA failure poisons the existing owner and is
not a retry promise. None of these APIs changes contact eligibility, mass,
source admission or the explicit nondegenerate geometry domain for OFF0.

The old initializer allocates no optional arena. Its device layouts and force
arithmetic remain unchanged. The private host facade gains one null ownership
pointer; its normal host forecast charges the changed object size. The new
raw device owner is noncopyable.

## Capacity and accounting

The optional limits default to 1,024 parents, 1 MiB device and 16 MiB host.
The explicit `Vehicle()` declaration allows 524,288 parents, 256 MiB device
and 512 MiB host. Optional bytes still have to fit the existing complete
family cap; these limits do not enlarge it. Count/byte checks precede optional
allocations. Reported host bytes count owned payload and named allocation
control allowances, not allocator overhead, driver memory or process RSS.
The embedded immutable handle and shared native backing are counted once;
shared backing is discounted only after the existing identity check.

On this binary64 host, the failure header is 32 B, policy 4 B, D1 8 B, and
failure state 208 B. Checked alignment gives 888 B for two parents. For the
original tire-free source counts (328,344 QEPH, 21,301 T3, 359,785 nodes), a
forecast with 1,024 curve points per family gives:

| Payload | Bytes |
| --- | ---: |
| Optional failure arenas, both families | 149,648,128 |
| Complete QEPH arena + mixed section + failure arena | 1,586,037,544 |
| Complete T3 arena + mixed section + failure arena | 102,322,956 |

These are layout forecasts, not full-vehicle mechanics or startup admission.
The complete nodal/contact/publication/other producer budgets remain separate.

## Owning qualification

Host author checks use one CPU and 512 MiB. Seven functions cover immutable
scope/canonical unused parameters, malformed and late declarations, exact byte
caps, rebasing, invalid flag encodings, and complete Q/T dispatch through
removal plus two nonzero post-removal intervals. Those dispatch tests use host
storage and do not claim an owner/CUDA execution.

```sh
cmake -S lib_utest/qualification/resident_shell_failure -B <build> -DCMAKE_BUILD_TYPE=Release
cmake --build <build> --target resident_shell_failure_host_check -j1
ctest --test-dir <build> -R '^resident_shell_failure_host$' --output-on-failure
```

The same seven host functions have the Bazel owner
`//lib_utest/qualification/resident_shell_failure:resident_shell_failure_host_check`.

The actual CUDA gate is authored for the parent-controlled queue:

```sh
cmake -S lib_utest/qualification/resident_shell_failure -B <cuda-build> \
  -DCMAKE_BUILD_TYPE=Release -DTL_RESIDENT_SHELL_FAILURE_CUDA=ON \
  -DCMAKE_CUDA_ARCHITECTURES=120
cmake --build <cuda-build> --target resident_shell_failure_cuda_check -j1
ctest --test-dir <cuda-build> -R '^resident_shell_failure_cuda$' --output-on-failure
```

Six actual-owner functions cover complete mixed families, first removal and
post-removal state against the independently invoked qualified value adapters,
exact disabled/active parity, other-family scope mismatch, a late final-parent
geometry failure and exact retry, failure-atomic readback including invalid
flag/nonfinite/device faults, bad common receipt/independent commit rejection,
and optional byte limits with clean initialization retry. The test-only linked
CUDA copy fault seam does not change production code.

The parent separately reruns existing mixed and legacy plastic resident gates
because shared kernel headers and host facade size changed. At author freeze,
only the host executable and temporary host syntax checks have run; there is
no new resident CUDA/native pass claim here. Qualified family value/native
force contracts are prerequisites, not substitutes for the owning runtime gate.
