# Parallel CIN recovery qualification

The production algorithm applies uniformly to every source admitted by the
existing CIN owner: unique secondary nodes, no secondary among any master slots,
and no rigid/CIN intersection. It does not select a vehicle, row count, node
order, geometry class, load magnitude, or epoch. Four native slots remain four
slots for both Q4 and repeated-third T3 patches. Admission and native arithmetic
are unchanged.

After successful ordinary and rigid advancement, a pure worker computes each
row's original `RecoverMotion` using the retained force-stage patch and current
master velocity/acceleration. An integer minimum identifies the earliest failed
source row. A second parallel pass writes all four secondary vectors only for
rows preceding that failure. Failed and later rows retain their original bytes.
The full existing serial dependent drift/orientation loop starts only if every
recovery row succeeded. Capture retains its original final control-Ok gate.
No floating-point sum is reassociated and no force or mass transfer is changed.

The separate typed scratch contains one 104-byte row and one shared 4-byte row
key. It is initialized on every admitted attempt, after prior-stage success;
prior rejection cannot consume old packets. No per-step allocation is added.
For the current 11,165-row V5 source the extra device allocation is 1,161,164 B.
`CinStorage` grows by 64 B, including both region records and both pointers.
Existing arena caps and exact-cap rejection tests remain in force. The private
null-tail comparison route keeps the complete serial recovery path.

Seven host functions compare all fields against the complete frozen old recovery,
including mixed and skewed geometry, source-row permutations, arbitrary domain
node order, 128-row boundaries, nonzero inputs, signed zero, subnormals, overflow,
first-row versus later failure, exact failed prefixes, retry and inclusive caps.
The four CUDA functions cover the complete old caller over multiple intervals,
earlier input/ordinary/rigid failures, injected post-kick recovery/drift failure
priority and capture, plus actual-owner last-secondary rotation failure and
accepted-state/capture/allocation-preserving retry. All floating comparisons are
bitwise. Tests do not relax native or owner admission. The multi-interval
fixture calls the existing Begin operation for each new epoch/attempt, rebuilds
fresh assembly loads/stiffness, and retains the prior accepted deformation and
coefficient history. A host control reproduces the StaleTrial rejection when
Accept is followed by advance without Begin, then verifies the corrected
lifecycle without resetting accepted state.

`reference/` preserves complete sources from 223d813. `Frozen.cu` and
`FrozenMotion.h` change only includes/namespaces; `FrozenTail.cuh` is the same old
completion body for fault injection after the kick. `recovery_proof.py` checks
exact extraction and reverses the scheduling-only owner delta before historical
CIN source proofs run. Old receipts and frozen donor files are retained.

The integration on `bf1c6a4` retains its optional, default-off limiter witness.
The checked owner reversal composes witness, recovery, force-transfer and group
steps; neither path removes an unrecognized delta. Both incoming manifest
histories remain under `reviewed_updates`. Limiter Control growth is already
part of this baseline and is not charged twice. The recovery delta remains
64 host bytes and 104 bytes per row plus a 4-byte key on the device. Combined
author checks include the limiter fixed-Control/seal-alignment and default
profile controls alongside the recovery and earlier CIN host gates.

Author validation uses one CPU and 512 MiB. C++-shaped syntax checks do not claim
NVCC, CUDA or native execution. Root owns actual hardware qualification and the
full-source throughput/equality comparison; no speedup is inferred from source.

Root owning gate (under the existing workstation guard):

```sh
cmake -S "$TL/lib_utest/qualification/cin_parallel_recovery" -B "$BUILD" \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_CUDA_ARCHITECTURES=120 \
  -DCIN_PARALLEL_RECOVERY_CUDA=ON
cmake --build "$BUILD" -j2
ctest --test-dir "$BUILD" --output-on-failure --output-junit "$REPORT.xml"
```

Bazel targets are
`//lib_utest/qualification/cin_parallel_recovery:host` and `:cuda`.
Retain the prior CIN input/ordinary/screen/group/force-transfer/capture,
physical-main, native trajectory and complete-owner forecast gates. The four old
layout-end assertions now explicitly include the recovery rows and key, while
preserving their exact-cap and minus-one controls.
