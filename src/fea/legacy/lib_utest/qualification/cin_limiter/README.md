# Actual successful CIN limiter

This adds an optional diagnostic to the existing post-CIN screen. The existing
`Result`, scalar formulas, strict minimum/tie order, selected step and rejection
semantics are unchanged. `capture_limiter` defaults to false and is valid only
with the named structural profile. When requested, the final screen winner is
copied into an appended pointer-free Control witness. A rigid winner reuses the
exact existing group trace once; no full second screen runs.

`CopyPreparedCinStructuralLimit` is a host-only query of the already synchronized
Control. It checks owner/token/phase, output alignment and complete retained-source
alias ranges before assigning its output. It is available on a successfully
prepared step at any epoch; commit, discard, a rejected advance and stale/foreign
tokens cannot publish it. It has no commit authority or device operation.

Memory growth is **nonzero and fixed**: `sizeof(cin_limiter::Witness)` is appended
to device Control and host Impl's Control copy. The layout test checks the normal
ABI and the exact seal-tail offset. All owner forecasts use `sizeof(Control)` /
`sizeof(Impl)` already. There is no new allocation, full-node array, query scratch,
extra CUDA copy or synchronization. Existing Control copies grow by the fixed
record. No memory/performance claim is inferred from source-only validation.

Authored tests: four host functions and three actual-owner CUDA functions cover
translation/rotation ties, ordinary and both rigid source kinds, unbounded/invalid
packets, exact enabled/default prepared-state equivalence over three accepted
intervals, repeated reads, foreign/stale/rejected/discarded attempts, source alias
and misalignment, exact caps, rollback and retry. Existing complete serial/native
screen and scheduling gates remain separate regression obligations. No numerical
test, compiler, native or GPU execution was performed by the source author.

Root gate, under the shared heavy guard:

```sh
cmake -S lib_utest/qualification/cin_limiter -B <new-cache> -DCIN_LIMITER_CUDA=ON -DCMAKE_BUILD_TYPE=Release -DCMAKE_CUDA_ARCHITECTURES=120
cmake --build <new-cache> --parallel 1 --target cin_limiter_host cin_limiter_cuda
ctest --test-dir <new-cache> --output-on-failure
```

Rebuild existing cached `cin_parallel_groups_cuda`, `cin_physical_timestep_cuda`,
`cin_physical_timestep_native` and `cin_physical_main_cuda` targets. Run their
existing CTests, plus the seal actual-owner gate. No new native donor is needed:
the numerical screen is unchanged. Bazel owning targets are
`//lib_utest/qualification/cin_limiter:{host,cuda}`.

Source-only entrypoint:

```sh
python3 -B lib_utest/qualification/cin_limiter/verify_sources.py
```

This also executes all five prior CIN scheduling identities and the seal identity.
Only reviewed changed records were advanced; prior hashes/bytes and the reason
remain in `reviewed_updates`. `witness_proof.py` removes exact additions before
the earlier complete mechanical proofs. Frozen reference files were not edited.
When combining with newer CIN force/recovery scheduling, apply this checked
reversal before the other scheduling proof and retain both reviewed histories.
