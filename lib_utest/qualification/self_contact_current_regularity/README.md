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

Author result (2026-09-12): all 14 host functions plus the source and public
syntax tests pass under that guard. The affected active-use qualifier also
passes all 17 host functions and its source proof. No Bazel or device/full-case
execution is claimed.
