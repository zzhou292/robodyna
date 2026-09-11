# TYPE45 immutable model

The model retains the actual complete rigid binding, its coefficient ledger and
domain. It copies joint properties and source geometry, checks source EID
uniqueness and exact endpoint/body kind associations, and maps axis-only N3
without assigning it a force or coefficient contribution. Plain/PART numeric
IDs may coincide; the named source kind is always compared.

The first physical profile requires rigid member endpoints and zero optional
free K/C for kinds 1/2/3. Its damping coefficients reuse RINI45_RB: initial body
mass and `(J1+J2+J3)/3`, with the same addition order. They remain distinct from
the later main-node minimum inertia and transferred scalar stiffness. No
`Reference`, automatic K, history, force cache, selected dt or owner claim is
manufactured by this module. The app must prove original source/default and
complete selected-joint coverage.

One HostArena holds the copied rows. Forecast includes the complete retained
binding backing once and the temporary shared source-identity index before any
borrowed row read. The index preserves source order; only complete success
publishes the model. Copy/move handles retain the complete immutable backing.

Five small host functions pass, covering three kinds, same numeric body IDs
with distinct roles, initial damping, source-order duplicate reporting, late
source/role rejection and retry, signed zeros, axis-only mapping, exact caps
before unreadable borrowed input, and backing lifetime. Author syntax and
configure pass; the small executable reused already built rigid-fixture/value
archives under the 1 CPU/512 MiB guard. Reports: `type45-model-build-1`,
`type45-model-tests-1`, `type45-model-syntax-2`, `type45-model-configure-1`.

Owning gates:

```
cmake -S lib_utest/qualification/type45_model -B BUILD -DCMAKE_BUILD_TYPE=Release
cmake --build BUILD -j2 --target type45_model_host
ctest --test-dir BUILD --output-on-failure -R type45_model_host
bazel test //lib_utest/qualification/type45_model:type45_model_host
```

No native/CUDA run or actual 38-joint source/model composition is claimed here.
The existing independent TYPE45 native three-kind values remain unchanged.
