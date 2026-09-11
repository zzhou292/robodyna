# Solid resident qualification

Author evidence: ten small host functions pass; the production and authored
test translation units pass the one-worker/512 MiB host syntax gate. CUDA files
in that gate have launch syntax stripped solely for C++ interface checking.
No author native, NVCC, CUDA or original-source execution has occurred.

The owning CUDA target links the existing independent complete native
Solid18/HEPH/S6Z oracles. It carries each native accepted history separately and
feeds only positions/velocities from the real prepared owner into the next
packet. All resident-retained history fields, source-slot RHS, material/HG
observations and native translational stiffness are compared using the existing
family-qualified field scales. Temporary native geometry and local forces are
not claimed as Batch readbacks. Exact sample/owner/attempt identity has no
floating tolerance.

The four owner tests cover eight real owner intervals, the native TT0 cache,
complete typed source/ledger/PART/plain/CIN attachment proof, exact three-family
RHS/STIFN scatter, untouched rotational channels, foreign claims and failed
borrows. A last-family material fault and a later contributor rejection preserve
every accepted solid and the actual owner snapshot; a new attempt succeeds.
Last-field readback NaN, overlapping output spans and wrong counts preserve all
caller outputs. Padding is compared only for failure preservation, never as a
successful history equivalence contract.

The optional original test constructs the shared existing 908 adhesive + 1309
mapped HEPH + 195 mapped S6Z fixture. One Batch arena at its exact forecast cap
must produce all2412 native constructor histories/RHS/STIFN. This is unclaimed
constructor qualification: public accepted readback remains `NotBound`, no full
vehicle owner or completed run is manufactured.

Owning commands (root workstation guard required):

```sh
cmake -S lib_utest/qualification/solid_resident -B BUILD_DIR \
  -DSOLID_RESIDENT_CUDA=ON -DSOLID_RESIDENT_ORIGINAL=ON \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_CUDA_ARCHITECTURES=120
cmake --build BUILD_DIR --target solid_resident_host solid_resident_cuda -j1
ctest --test-dir BUILD_DIR --output-on-failure -R '^solid_resident_'
```

The reused startup subdirectory also supplies its unchanged constructor and
legacy-family regression targets. Re-run its selected checks when a native gate
finds a shared constructor/formula concern. Owning Bazel targets are
`//lib_src/elements/solids/resident:batch` and
`//lib_utest/qualification/solid_resident:host_check`.
