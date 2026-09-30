# Vehicle-sized host plasticity catalog qualification

This is host startup/catalog qualification only. The source-sized fixture uses
synthetic native squares/triangles at the selected no-tire inventory counts
(328344 QEPH + 21301 T3, 359785 nodes) and 875 synthetic definitions. It does not
import the actual Yaris materials, execute native constitutive code, allocate
CUDA state, or prove full-vehicle mechanics/contact support.

Production API: `ShellBatchPlasticityBinding::InitializeCatalog` with a separate
`ShellPlasticityCatalogLimits::Vehicle()` profile. The old `Initialize` API keeps
its 1024-parent/2048-node rejection and byte-admission semantics. The shared
binding/resident constants and the total 1024-point curve pool are unchanged.

The six small functions cover:

- Count/byte/overflow rejection before poisoned borrowed ranges, exact-cap retry,
  and legacy large-byte-budget/ignored binding-scratch semantics.
- Original source-order first failure, unsorted wide IDs and exact native bits.
- Last parent/source/material/section failure, invalid curve/pool and retry.
- All 16 retained/transient backing allocations in a >1024 Q/T fixture.
- 1024 material/section definitions, mixed and all-analytic zero-pool ownership.
- Complete small parent mapping, immutable copy/move and destroyed source lifetime.

The separate source-sized function checks all 349645 parent mappings/parameters,
complete inventory ownership, exact late failure/retry, and emits measured
`vehicle_catalog_owned_bytes` and `vehicle_catalog_startup_scratch_bytes`.

Configure/build without CUDA or Fortran:

```sh
cmake -S lib_utest/qualification/vehicle_plasticity_catalog -B BUILD -DCMAKE_BUILD_TYPE=Release
cmake --build BUILD --target vehicle_plasticity_catalog_check shell_plasticity_binding_check host_shell_collection_check vehicle_shell_host_check -j1
ctest --test-dir BUILD -R '^(vehicle_plasticity_catalog_small|shell_plasticity_binding_check|host_shell_collection_check|vehicle_shell_host_small)$' --output-on-failure
```

The full fixture needs a separately scheduled larger memory budget (it retains
large native reference geometry as well as catalog input/outputs). Do not run it
under the author's 512 MiB lightweight allowance:

```sh
ctest --test-dir BUILD -R '^vehicle_plasticity_catalog_source_size$' --output-on-failure
```

Bazel owners are `//lib_utest:vehicle_plasticity_catalog_small` and the explicitly
manual `//lib_utest:vehicle_plasticity_catalog_source_size` target. The existing
`//lib_utest/qualification/plasticity_binding:shell_plasticity_binding_check`
remains a legacy/mixed catalog gate.

Author evidence is reported in the handoff; source-sized and owning Bazel gates
remain with the root workstation queue. No performance or native constitutive
accuracy claim is based on these host identity/storage checks.
