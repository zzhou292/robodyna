# Mapped QEPH compact activity validation

The measured full V5 query copied 648,188,000 bytes to produce 324,094 flags.
The full force cache accounted for 461,509,856 bytes and approximately 71–73 ms
of copy/validation per query. The source report is
`crash-work/reports/qeph-activity-profile-analysis-1.{json,md}`. This qualifier
does not claim a new measured runtime until root execution.

Only mapped `CopyAcceptedParentActivity` and `CopyPreparedParentActivity` select
the new path. The existing section and failure readbacks still run freshly,
with the same finite, encoding, saved-history, tagged-union and source checks.
Every complete force result is then freshly checked on CUDA with the unchanged
host/device `mapped::ValidResult`. An integer minimum selects the first invalid
parent. Only after that phase passes does the original host role/activity loop
inspect the freshly staged section/failure data in source-parent order. The
host role loop remains deliberately unchanged in schedule; this increment does
not move section packing or failure validation onto CUDA.

The new optional mapped tail holds four status bytes followed by one byte per
parent. The mapped startup forecasts its complete device extent and host
readback backing, including the extra 64-byte shared-control reservation.
Legacy arenas have no activity tail. The caller never observes scratch, a
partial mask or a cached validation verdict. Every query resets the private
integer status and rewrites valid parent bytes on the owner's stream. CUDA
failure retains the existing Runtime poisoning; prepared validation failure
still discards the candidate. Public preflight, ownership, token, diagnostics,
source-range and output-alias checks remain in place. Full-history and full
force readbacks retain their original path.

At 324,094 parents the compact packet is 324,098 bytes. The four D2H calls and
three existing synchronization boundaries remain, but total payload becomes
187,002,242 bytes/query: the unchanged 186,678,144 bytes of sections/failure plus
the compact packet. No floating-point reductions, forces, history updates or
owner publication changes are introduced.

## Source identity boundary

`mapped::BuildModel` copies each reference directly from the immutable physical
binding and records the original parent mapping. The sidecar initializer copies
the law from the same admitted complete catalog. Public APIs expose neither a
mutable resident model nor a mutable catalog/reference. These retained bindings
and uploaded source records do not change on commit/discard. The kernel uses
those already authenticated uploaded references/roles, as the mapped force and
assembly kernels do. Its result/history/epoch tests run fresh on every query;
it never trusts a previous success flag. This is not a promise to detect
arbitrary external corruption of immutable device model/source storage.

`FrozenReadback.cpp.txt` is the full baseline `mapped/Readback.cpp` from
`14ab5248217d225cfefc992f2c3d8b7915b151ca`. `SerialValidation.h` contains its two
complete validation functions, with only explicit host-oracle State
qualification. The source verifier regenerates those exact functions, checks
the complete baseline hash and authenticates all owning/reused files.

## Tests and execution boundary

The host tests cover exact compact layout and inclusive caps, full mapped host
budget rejection/retry, the frozen global force-before-role error order, and
the default-preserving fresh ReadFailure callback/error phases. CUDA tests use
the existing six-family physical owner/CIN fixture for six common commits,
accepted/prepared/repeated reads, an explicit PART skin, late force failures,
candidate discard/retry, unchanged full readbacks and alias/stale-input
rejections. A fixed transfer observer proves no full ForceTrial D2H remains in
activity calls. Device-cache controls test active 0/1/257/−1/NaN before narrowing;
the inactive values are explicitly prescribed validation controls, not a new
material-removal trajectory claim.

Root owning gate (run through the normal root guard):

```sh
cmake -S "$TL/lib_utest/qualification/qeph_mapped_activity" -B "$BUILD" \
  -DCMAKE_BUILD_TYPE=Release -DQEPH_ACTIVITY_CUDA=ON -DCMAKE_CUDA_ARCHITECTURES=120
cmake --build "$BUILD" --parallel 4
ctest --test-dir "$BUILD" --output-on-failure
```

The dedicated CMake includes the existing complete physical publication
regressions. Affected owning gates also include `qt_mapped`,
`shell_parent_activity`, `resident_shell_failure`, and `qeph_mapped_gather`.
The complete full V5 loaded-prefix gate must retain the manifest
`752a19b914d2e515866925b71cef9139d14eb700d9e7d9738ee9ab2d5c248d45` and viewer
receipt `56334f517494ff9f0e98b711a183cacd4e6a96114a9173cb2a91d2533f366eec`
for the same original two-interval input. The author runs only guarded
one-CPU/512 MiB host/source/syntax checks; no author native/NVCC/CUDA execution.
