# T3 activity readback within one serialized call

This removes a duplicated complete force-cache transfer only from mapped parent
activity when a true one-point sidecar is present. It is uniform across all
admitted counts and law mixtures; no case identity or constant activity is used.
Public full-history and standalone readbacks retain their original fresh reads.
No history, force, tolerance, source admission, allocation, forecast, clock or
publication rule changes.

Before this change, accepted/prepared activity used this sequence:

1. Pending error, fresh one-point/mixed/failure sections.
2. `ValidateMappedSections`: fresh complete `ReadResults`, including mapped
   force/history/reference/endpoint validation, then parent-ordered role/activity.
3. `ValidateOnePointReadback`: source preflight, another complete `ReadResults`,
   then the exact one-point saved/current/history/reference/time/epoch checks.
4. Success-only flag and diagnostic publication.

The private combined `ReadParentActivity` confines reuse to the successful
mapped-validation callback of `ReadFailure`. Only const host checks intervene
between the synchronized copy and its use. `T3Batch.h` requires serialized calls
on the owner stream; its private arrays have no independent admitted writer.
There is no retained freshness bit, general staged-read API or cross-call cache.
The second pending-error check remains at its original point. The file-local
one-point helper preserves every prior predicate and evaluation order.

Nonmapped `ValidateMappedSections` does not copy results. Those callers therefore
keep `ValidateOnePointReadback` and its fresh transfer. The full layered-section
API has a different original order (point identity before mapped role agreement);
it is deliberately unchanged. All accepted/prepared alias, owner, token, capacity,
null-output and stale-receipt checks stay ahead of transfers. Numerical candidate
failure discards; device failure poisons; caller output writes still require
complete success. A failed DMA that no longer exists is not an observable failure
site; every remaining DMA is independently exercised by the new CUDA tests.

For any affected family the reduction is `N * sizeof(t3::ForceTrial)` bytes,
one D2H copy, one stream synchronization and one repeated full mapped result
validation. At the measured V5 count of 21,301 and this ABI's 984-byte ForceTrial:

| Payload | Before | After |
| --- | ---: | ---: |
| T3 activity bytes/query | 58,961,168 | 38,000,984 |
| T3 copies/query | 6 | 5 |
| T3 synchronizations/query | 5 | 4 |
| All-family bytes/step after the QEPH mixed slice, three queries | 220,804,386 | 157,923,834 |

The 62,880,552-byte per-step reduction is a transfer census, not a measured
speedup. Source admission and other role counts remain unrestricted within
existing profiles/caps.

`Frozen*.txt` retain the complete prior owning sources at `bf1c6a4`. The verifier
checks the exact helper extraction and complete public-call substitution, and
proves the mapped checker, transport body and shared `ReadFailure` are unchanged.
Generated host flows use only namespace/state/transport decoration of the full
validation bodies. Their transport/source-observation seams are explicit; they
do not claim native execution or a second owner. Six host functions cover all
four admitted roles, counts 1/2/7/129, mapped/nonmapped paths, stage ordering,
source/point identity and repeated-query freshness. Six CUDA functions exercise
the actual common owner and public APIs, six carried intervals, frozen-call
comparison, transfer counts, malformed point encoding, each remaining copy
failure, alias/stale/null/cap preflight and candidate discard/retry. Existing
`shell_parent_activity` and mapped Q/T gates retain removal, full-history and
rigid-skin coverage.

Author checks use one CPU/512 MiB host and syntax only. Root commands:

```sh
cmake -S Total-Lagrangian-FEA/lib_utest/qualification/t3_activity_readback \
  -B crash-work/build/t3-activity-readback-root-1 \
  -DCMAKE_BUILD_TYPE=Release -DT3_ACTIVITY_READBACK_CUDA=ON \
  -DCMAKE_CUDA_ARCHITECTURES=120
cmake --build crash-work/build/t3-activity-readback-root-1 --parallel 4
ctest --test-dir crash-work/build/t3-activity-readback-root-1 --output-on-failure
```

Affected gates: `shell_parent_activity`, `qt_mapped`, `t3_one_point_resident`,
`resident_shell_failure`, then complete V5 archive equivalence and timing.
