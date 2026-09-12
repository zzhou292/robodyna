# Fresh solid candidate validation scheduling

This increment moves only the complete existing `ValidResult` predicate from the
serial candidate measurement to one fresh byte per parent on the same stream.
It applies uniformly to every currently admitted count/profile in all five solid
families, including tabulated/analytic LAW44 and both LAW90 loading flags. It does
not change material/element calculations, assembly, public readback, source
admission, owner publication, tolerances or precision.

The baseline is TL `dfa0a93`. Its whole `Measure.h`, `Candidate.cu`,
`ResultChecks.h`, `Arena.h` and `Arena.cpp` are frozen in `frozen/`. The build
adapts only include paths and namespaces for the independent old caller. Source
verification additionally checks that the current initialization/update bodies,
complete predicates, finalizer identity and ordered work fold remain literal.

## Scheduling and failure contract

1. Existing typed initialization/update kernels finish in the owner stream.
2. Five typed validation kernels overwrite every active byte, including rejected
   rows. A nonzero existing element status returns zero before any reference,
   material or trial-history dereference. No first-error reduction replaces the
   byte array. Empty families consume no storage or values.
3. The original one-thread finalizer consumes each byte at the original predicate
   position, with unchanged family/parent/slot order. Earlier cumulative work
   overflow therefore still beats an invalid later parent or later family.
   Accepted RHS forces, signed sums, native minimum and partial error diagnostics
   retain their original operations.

Validation uses the explicit trial, `identity.time` and `identity.epoch` passed to
that finalizer; initial construction uses its original zero/zero identity. It
does not read the previous control diagnostics. The bytes have one stream-local
lifetime from this fresh launch to its immediately following finalizer; there is
no retained verdict or new public staging API. Subsequent calls always overwrite
them. Existing `ReadResults` remains a separate fresh full validation/transport.

Five `uint8_t` regions are appended through `BoundedArenaLayout`, constructed in
the original upload arena and rebased with the typed family. The whole original
arena range already participates in output alias rejection. Actual header,
layout, alignment and upload bytes remain charged by `Batch::Forecast`; there is
no additional allocation or D2H transfer. For the documented count-only fixture
908/1991/350/386/1345 (not a new source admission), the owning host layout changes
115345888 to 115350920 bytes: 5032 bytes including 40 header bytes and alignment.
Flags may occupy some existing padding; N alone is not an exact arena delta.

## Qualification

Host tests compare all five predicates and exact raw flag encodings, unavailable
failed histories, explicit/stale diagnostic identities, analytic/tabulated rear
materials, signed/cancelling accepted-RHS work, same/cross-family overflow and
late errors. Budget tests exercise actual aligned regions, legacy empty families,
one-byte-short host/device caps, rejected overflow and preserved outputs.

Root CUDA tests compile the complete current and frozen callers in one executable
and compare every history/cache, status and diagnostic through TT0 and eight
carried intervals for both foam flags and both rear material forms. Further gates
exercise all-family early/late force/history/status faults, cumulative overflow
before a final foam fault, failed-trial accepted preservation, retry, explicit
identity and more rows than validation workers with unavailable failed histories.
Existing extended resident owner/native tests remain required for public
alias/poison/discard/commit behavior; this qualifier does not invent another owner.

Author work is host/source/shape only under one CPU/512 MiB. Root commands:

```
cmake -S Total-Lagrangian-FEA/lib_utest/qualification/solid_candidate_validation \
  -B crash-work/build/solid-candidate-validation-root-1 \
  -DCMAKE_BUILD_TYPE=Release -DTL_SOLID_CANDIDATE_VALIDATION_CUDA=ON \
  -DCMAKE_CUDA_ARCHITECTURES=120
cmake --build crash-work/build/solid-candidate-validation-root-1 --parallel 4
ctest --test-dir crash-work/build/solid-candidate-validation-root-1 --output-on-failure
```

Also rebuild/run the owning `extended_solid_resident` (including original and
analytic resident gates) and legacy `solid_resident` caches. Full V5 identical
archive/output comparison and stage timing remain root work. The prior ~133 ms
inclusive solid evaluate measurement does not isolate this predicate, so no
speedup is claimed before that measurement.
