# Explicit filtered LAW44 with zero strengthening

`ShellPlasticityRatePolicy::FilteredZeroC` admits enabled=true, C=0, resolved
P=1 and a positive, explicitly supplied cutoff. It preserves the native rate
history even though the strengthening multiplier is exactly one. The default
`Legacy` policy preserves positive-C/P admission and disabled zero-history
behavior. No constitutive return map, NIP3 reduction, owner or clock is added.

Pinned OpenRadioss: `a62b27e6baa555d222a580d6218867d0be4d70b5`.
The existing complete, authenticated LAW44 point reference remains the oracle.
`hm_read_mat44.F:178-200,208-225` resolves CC=0, CP=1, VP2 and filtered rate.
`SIGEPS44C:204-222` updates filtered UVAR independently of CC; :266-267 skips
strengthening for CC0. This also applies to parent OFF0, whose plastic and
thickness changes retain the existing point activity gates.

The original Yaris glass inputs are E70GPa, nu.22, rho2500kg/m3, SIGY30MPa and
ETAN1GPa. Their exact analytic converter expression is ETAN*E/(E-ETAN).
The tests use those values and the resolved 10000Hz cutoff. A separate table
control qualifies reuse beyond the analytic branch; it is explicitly synthetic.
Source MAT123 failure and NUMINT1 interpretation are not admitted by this patch.
The original blank P is resolved by the app; this API does not guess it.

The native point packet accepts one optional ISRATE override. Prior callers
omit it and retain their original mapping. The new bridge supplies ISRATE1
and CC0/CP1, then calls complete native SIGEPS44C. Native donor bytes, hashes,
preparation logic and all previous C entrypoints remain unchanged. No reference
formula is replaced by a production helper.

Three host functions cover filter independence, inactive-parent filtering,
invalid policies/prepared fields, late failure atomicity and distinct NIP3
histories. Two additional catalog functions verify typed parameter queries,
retained ownership/scope and late-policy rejection with retry. Two native
functions compare independent loading/unloading/filter saturation/OFF0
histories and strengthening independence from a large finite filtered rate.
One CUDA function prepares parameters on device and advances its own histories
for both hardening branches, with a late rejected update and exact-state retry.
There is no glass fracture, full-vehicle admission or trajectory claim.

Author gate: 3 values + 18 catalog functions passed under 1 CPU / 512MiB.
Native C++ and CUDA-shaped host syntax passed; these are not Fortran/CUDA
execution. Reports are `crash-work/reports/shell-filtered-zero-c-author-{1,2}`.
The first configure attempt selected unavailable Ninja; standard Unix Makefiles
configuration passed without a source change. Native and real CUDA execution
are scheduled by the root agent, on this frozen worktree.

Root owning gate (substitute new build directory and configured CUDA architecture):

```sh
cmake -S lib_utest/qualification/shell_filtered_zero_c -B BUILD_DIR \
  -DCMAKE_BUILD_TYPE=Release -DTL_FILTERED_ZERO_C_NATIVE=ON \
  -DTL_FILTERED_ZERO_C_CUDA=ON -DCMAKE_CUDA_ARCHITECTURES=120
cmake --build BUILD_DIR --parallel 2 --target \
  shell_filtered_zero_c_values_check shell_filtered_zero_c_native_check \
  shell_filtered_zero_c_cuda_check shell_plasticity_binding_check \
  law44_point_native_check law44_rate_native_check law44_analytic_native_check
ctest --test-dir BUILD_DIR --output-on-failure -j1 \
  -R '^(shell_filtered_zero_c_(values|native|cuda)|shell_plasticity_binding_check|law44_(point|rate|analytic)_native)$'
```

Owning Bazel host targets are
`//lib_utest/qualification/shell_filtered_zero_c:shell_filtered_zero_c_values_check`
and `//lib_utest/qualification/plasticity_binding:shell_plasticity_binding_check`.
Existing resident layouts derive storage from the parameter type's actual size;
this trailing policy is part of exact material scope in both catalog and
homogeneous storage. No runtime resident qualification is claimed here.
