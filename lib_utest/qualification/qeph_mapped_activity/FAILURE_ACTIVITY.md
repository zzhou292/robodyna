# Fresh mapped QEPH failure activity

This increment replaces only the mapped activity query's full failure-sidecar
transfer and host validation. The qualified force-cache compact kernel remains
unchanged. Direct full section/failure/result APIs retain their path; the source
verifier checks the complete old `FailureHostStorage::Read` body byte-for-byte
and the full failure-value implementation apart from the added host/device
annotation on `ValidFailureEncoding`.

Every activity query performs these ordered phases:

1. Existing public owner/epoch/capacity/output-alias preflight.
2. Fresh complete mixed material-section copies, finite validation and typed
   packing, followed by the existing failure shape/source-availability checks.
3. One GPU worker per parent validates the actual selected failure slab against
   its immutable uploaded policy and the same saved plastic section. The exact
   `ValidFailureEncoding` and `ValidFailureState` helpers check all three points,
   current/saved stresses, failure times, damage, parent activity, and None,
   constant-all-points or TAB1-any-point policy. Raw bool bytes are checked before
   bool evaluation. Integer minimum retains first-parent ordering internally;
   the public failure report keeps its original message/status and unset index.
4. Copy and synchronize a four-byte status plus one validated failure flag per
   parent. A host byte check also rejects corrupted compact flags before use.
5. Reuse that same device packet for the existing fresh full-force validation.
   Its independent host packet is copied/synchronized exactly as before.
6. Check the freshly typed section roles and the two validated activity fields
   in original parent order. Publish flags and diagnostics only on full success.

There is no retained validity verdict. Failure packet lifetime ends on the GPU
only after its synchronized host copy. The host failure flags remain separate
while force validation overwrites the device packet. No activity result reads
the older full host failure staging. A direct full-history query refreshes that
staging with its original tagged-union restoration and full validation.

The admitted physical catalog/reference/failure binding is immutable and retained
by the batch. The existing sidecar initializer uploads each failure policy and
section law from that exact source; commit/discard never changes these records.
The copied material sections and subsequently read device saved sections belong
to the same selected slab and exclusive owner stream. This is the same immutable
uploaded-source contract as the qualified force activity path, not a promise to
detect arbitrary outside corruption of private source/model memory. QEPH does
not admit T3's one-point role; the true T3 readback path is unchanged.

No device storage is added. The existing mapped packet is reused sequentially.
One new startup-sized host byte buffer is charged by the full forecast, including
its 64-byte shared-control reservation and the larger private Impl. At 324,094
parents the buffer/control charge is 324,162 bytes; private-object sizing remains
the actual `sizeof(Impl)`. Its entire byte range participates in output-alias
rejection. No per-query allocation or additional clock is introduced. Device
errors retain Runtime poisoning; prepared validation failure discards only the
candidate, and successful retry uses fresh packets.

## Transfer evidence and qualification

`TransferBudget.json` records the actual V5 source counts and current sizeof
payloads. Before this increment, accepted-witness activity transfers 259,631,410
bytes across 11 D2H copies and nine sync boundaries: QEPH 187,002,242, true T3
58,961,168, and QBAT 13,668,000 bytes. T3 still reads its 20,960,184-byte force
cache twice; it is a separate follow-on. Common diagnostics and roster mapping
are host operations and add no D2H payload to this census.

The new QEPH query transfers 114,729,284 bytes, reducing each query by 72,272,958
bytes. Four QEPH copies and three sync boundaries remain. Overall witness payload
becomes 187,358,452 bytes. The old measured failure leg was 22.550 ms accepted and
22.295 ms prepared in `qeph-activity-profile-analysis-1`; these are historical
measurements, not a new speedup claim. Actual timing and archive equality remain
root qualification gates.

Host controls compare 30 policy/encoding cases with the complete frozen failure
values, check every point/stress channel, and exercise inclusive arena and host
forecast limits. New CUDA controls cover 257 parents, earliest/late errors,
None/constant/TAB1/skin roles, section-before-failure-before-force priority,
accepted/prepared atomic outputs, compact raw 2/255 corruption, aliases,
poisoning, discard/retry and subsequent full reads. The transfer observer checks
exactly two compact copies, no full failure/force copy for mapped activity, and
the unchanged full failure readback path. Existing owner tests keep six loaded
commits and repeated accepted/prepared observations.

Run the owning CMake recipe in `README.md` with `QEPH_ACTIVITY_CUDA=ON`. Rebuild
`resident_shell_failure`, `shell_parent_activity`, `qt_mapped` and mapped QEPH
regressions, then compare the full V5 archive under the exact baseline inputs.
Author checks are one CPU/512 MiB host/source/CUDA-shaped syntax only; actual CUDA
execution and timing are root-owned. The previous force-only source manifest is
retained unchanged as `force-source-manifest.json`.
