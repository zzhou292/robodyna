# Owning TYPE13 resident qualification

Host-only author checks use `TYPE13_RESIDENT_NATIVE=OFF` (default) and make no
CUDA calls. Root schedules the native and actual device gates:

```sh
cmake -S lib_utest/qualification/type13_resident -B <build> \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_CUDA_COMPILER=/usr/local/cuda/bin/nvcc \
  -DCMAKE_CUDA_ARCHITECTURES=120 \
  -DTYPE13_RESIDENT_NATIVE=ON \
  -DTYPE13_RESIDENT_ORIGINAL=ON \
  -DTYPE13_RESIDENT_CUDA=ON
cmake --build <build> --parallel 1
ctest --test-dir <build> --output-on-failure
```

Run these through the workstation guard; author execution remains limited to
one CPU/512 MiB, while root owns Fortran/CUDA/original-count execution. Owning
targets are `type13_resident_host_test`, `type13_resident_native_test`,
`type13_resident_original_test` and `type13_resident_cuda_test`. Tests do not skip
when CUDA is unavailable. The source and original fixture identity tests use
the existing pinned declarations and donor verifier.

The original fixture is included from `type13_model/OriginalSource.h`; no source
deck, nodes, properties or beam arrays are duplicated. Native packet agreement
uses the unchanged existing TYPE13 `2e-11 * max(1e-10, magnitude)` budget.
Assembly comparisons sum this allowance over the incident node/component terms
and add their ordered binary64 reduction bound, preserving cancellation cases
without borrowing whole-system force or energy magnitudes.

`PublicationPeer.h` exists only in this qualification. It calls the real sole
owner commit and private preflight/switch; there is no production standalone
publisher. The CIN case's non-TYPE13 coefficients, loads and witnesses are
explicit synthetic qualification inputs. Full vehicle source/ledger and common
mapped publication remain separate integration obligations.
