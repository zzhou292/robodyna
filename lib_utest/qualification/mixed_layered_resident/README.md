# Mixed layered resident qualification

Host-only default:

```sh
cmake -S lib_utest/qualification/mixed_layered_resident -B build/mixed-layered-host -DCMAKE_BUILD_TYPE=Release
cmake --build build/mixed-layered-host --target mixed_layered_layout_check -j1
ctest --test-dir build/mixed-layered-host -R '^mixed_layered_layout_check$' --output-on-failure
```

The host executable performs no CUDA call; linking the runtime permits testing
the actual forecast owner without a production hook. Author gate: all three
functions pass under one CPU/512 MiB; `/tmp/tl-mixed-layered-layout-1.xml`.

Root-owned native/CUDA gate (not run by the author):

```sh
cmake -S lib_utest/qualification/mixed_layered_resident -B build/mixed-layered-resident -DCMAKE_BUILD_TYPE=Release -DCMAKE_CUDA_ARCHITECTURES=120 -DTL_MIXED_LAYERED_RESIDENT_CUDA=ON
cmake --build build/mixed-layered-resident --target mixed_layered_layout_check mixed_layered_resident_check resident_plasticity_check shell_plasticity_binding_check -j2
ctest --test-dir build/mixed-layered-resident -R '^(mixed_layered_layout_check|mixed_layered_resident_check|resident_plasticity_check|shell_plasticity_binding_check)$' --output-on-failure
```

The native fixture imports contain CUDA launches, so every file using that
fixture is compiled as CUDA. Pure production storage/readback/dispatch headers
passed separate host syntax. No author claim of CUDA compile/runtime success.
The older native material/force gate is unchanged and remains separate.

Bazel owns the host layout check and production dependencies via
`//lib_utest/qualification/mixed_layered_resident:mixed_layered_layout_check`;
the native mixed transaction gate uses the established CMake native owners.
