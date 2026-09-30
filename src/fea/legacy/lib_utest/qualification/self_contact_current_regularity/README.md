# Current self-contact regularity host qualifier

This isolated host target retains `SelfContactActiveUseBinding`, evaluates the
actual current `VectorView`, certifies every base-active Q4/T3 parent and every
fixed represented facet, and publishes a fresh typed receipt. It has no force,
broadphase, crossing, owner clock, adaptive refinement, or timestep role.

The author gate is intentionally host/source/syntax only:

```bash
cmake -S lib_utest/qualification/self_contact_current_regularity \
  -B /tmp/self-contact-current-regularity -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/self-contact-current-regularity --parallel 1
ctest --test-dir /tmp/self-contact-current-regularity --output-on-failure
```

Run it under `crash-work/reports/connection-author.lock`, one CPU, and a
512 MiB address-space limit. Bazel wiring is authored for root source
composition but is not an author gate. Root still owns Bazel/full-source/native
runtime integration; this module has no CUDA/NVCC/GPU/Fortran target.

The V5 performance profile uses one authenticated facet cursor during source
validation, retains only the level-local Q4/T3 weight templates, and applies
the certified directed-measure interval as a fast exact-sign filter. Runtime
remains serial so first-failure parent/facet order is unchanged. The former
per-facet `CurrentFixedTriangle` arena is absent.

Author result (2026-09-13): all 17 host functions plus source and public syntax
tests pass under that guard. On one CPU, representative 15:1 Q4:T3 samples at
512/1,024/2,048 parents took 2,426/4,447/8,989 us for initialize plus certify,
versus 9,201/29,698/96,715 us before the optimization. Arena forecasts fell
from 808,960/1,617,920/3,235,840 bytes to
237,856/475,424/950,560 bytes. With this ABI's 232-byte parent result and
288-byte level-0 templates, the exact incremental regularity arena forecast for
337,092 parents is 156,410,976 bytes. This is a synthetic forecast, not a
claim that the full V5 application gate passed; the parent integration owner
must run that gate.
