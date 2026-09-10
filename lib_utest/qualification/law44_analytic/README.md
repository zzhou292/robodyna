# Analytic LAW44 qualification

The host-only gate has no CUDA or Fortran requirement:

```
cmake -S lib_utest/qualification/law44_analytic -B BUILD \
  -DTL_LAW44_ANALYTIC_NATIVE=OFF -DCMAKE_BUILD_TYPE=Release
cmake --build BUILD --target law44_analytic_preparation_check shell_plasticity_binding_check -j1
ctest --test-dir BUILD -R '^(law44_analytic_preparation_check|shell_plasticity_binding_check)$' --output-on-failure
```

For the independent native oracle, omit the OFF option or set it ON, then build
`law44_analytic_native_check`, `law44_point_native_check` and
`law44_rate_native_check`. These run the unchanged MFUNC0 analytic and MFUNC1
tabulated donor. Source conversion, special virgin slope, independently carried
load/hold/reverse history, actual physical thickness, NIP3 separate WF/WM, native
work and late failed output are checked. Comparison budgets are the existing
native point/physical-thickness budgets; no fitted material tolerance is added.

For actual CUDA point/native parity, add `-DTL_LAW44_ANALYTIC_CUDA=ON
-DCMAKE_CUDA_ARCHITECTURES=120`, build/run `law44_analytic_cuda_check`. Source
parameters are prepared on the device, followed by 384 native-compared steps
for each original coefficient combination and a late arithmetic rejection.

The existing joined resident gate now has its own standalone configuration:

```
cmake -S lib_utest/qualification/resident_plasticity -B RESIDENT \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_CUDA_ARCHITECTURES=120
cmake --build RESIDENT --target resident_plasticity_check -j4
ctest --test-dir RESIDENT -R '^resident_plasticity_check$' --output-on-failure
```

It uses the existing synthetic mixed-shell fixture, not a full Yaris run. Both
mixed table/analytic and all-analytic catalogs must yield on actual CUDA, retain
null curve pointers for the analytic path, preserve accepted fields/material
histories after a late T3 failure, retry exactly and keep allocations stable.
Existing table-only resident cases also run. Root schedules native/CUDA/heavy
builds under the workstation guard.

Bazel host ownership remains explicit:
`//lib_utest/qualification/law44_analytic:law44_analytic_preparation_check` and
`//lib_utest/qualification/plasticity_binding:shell_plasticity_binding_check`.
Native Fortran and actual point/resident CUDA qualification use the owning CMake
targets above; there is no new external solver/runtime dependency.

Follow-on robo-dyna edits after these gates pass: add a separate strict original
MAT024 analytic declaration/parser (explicit LCSS0 and VP0, blank inline, FAIL,
TDEL and LCSR, positive C/P/SIGY and admissible ETAN); retain original cards,
nullable fields, source IDs and SI conversion; teach the source material catalog
builder to emit the trailing tag/linear values with curve_id0 and omit only
nonexistent curve records. Preserve the old table parser and archived JSON.
The full-shell coverage compiler must continue to distinguish declared from
runtime-admitted source cards. No app files change in this TL patch.
