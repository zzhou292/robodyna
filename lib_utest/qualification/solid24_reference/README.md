# Native HEPH brick startup

**Source-profile boundary:** the original rubber HGID2000017 has IHQ2/QM0.1.
The converter applies it after the blank section and changes Isolid24 to
Isolid1 (`convertprops.cxx:221–253`). Therefore this module is a qualified
HEPH value implementation on original geometry, not the original selected
bushing startup. The full-shell demo explicitly selects HEPH24/S6Z under
`planning/YARIS_RUBBER_SOURCE_PROFILE.md`, preserving source controls separately.
No vehicle participant uses it yet. Original Isolid1 emulation is deferred;
the exact hourglass receipt is
`crash-work/reports/yaris-rubber-hourglass-source-1.json`.

`solid24` provides a bounded value entry for the eight-node JHBE24 startup:
immutable original identities, SRCOOR3 orientation and cyclic frame, SZDERI3
center volume, SDLEN3 characteristic length and SMASS3 translational mass.
It reuses the unchanged shared brick-frame utility. The three production
headers separate types, geometry and reference construction.

This is the SINIT3 startup invocation selected after INITIA's local frame
override. It takes the caller-prepared virgin density and FILL1; it does not
resolve material defaults, authorize source force participation, produce
constitutive history or certify a stable timestep. Unsupported profiles,
repeated node IDs, invalid geometry or nonfinite results leave output untouched.
The first profile has no scalar rotational inertia, ALE or reference shape.

The original rubber fixture contains1504 cells/2308 nodes in8 parts. All1309
bricks match the explicitly selected HEPH native reference on CPU and CUDA, including source
slots, volume, length and equal per-slot masses. Their startup mass subtotal is
0.781662kg for that selected profile. The195 wedges explicitly reject this entry: INITIA dispatches them
to S6ZINIT3 before the HEPH branch, with their own connectivity and mass rules.
They remain required load paths in the vehicle plan.

The source fixture is shared under `../solid_common/source_fixture/`; it
retains raw coordinate bits, canonical SI, original EID/PID, line/mask records,
all8 part/section/material cards and authenticated archive/array receipts.
The source collector reused modelio's bounded reader and field parser; its
reproducible acquisition evidence is in
`crash-work/reports/yaris-rubber-source-fixture-1/`. The fixture identity check
pins both the493851-byte header and17991-byte manifest.

The native manifest authenticates20 complete donor/include files from pinned
OpenRadioss `a62b27e6baa555d222a580d6218867d0be4d70b5`. Common frame/mass donors
are referenced by their existing paths. Only private symbol namespaces and
exact SRCOOR3 statement extraction occur; no numerical expressions are patched.
The shared dormant ALE/Q1NP mass context contains no mechanics. Complete
SZDERI3, SDLEN3, SLEN and SMASS3 bodies run independently of the C++ values.

The SI comparison uses2e-11 relative plus64epsilon times each dimensional group.
Original-mm outputs use a separate world-coordinate/length conditioning
allowance. EID2191071's frame component cancellation is documented in
`crash-work/reports/solid24-unit-diagnosis-1.md`; physical frame and mass
perturbations reject. SI equations and their original comparison are unchanged.

Root gate `solid24-reference-root-tests-3` passes10 numerical functions:
3 host,3 small native,2 original-source and2 CUDA, plus2 source identities.
It includes the positive-volume SDLEN3 small-face EP03 branch, reverse
orientation, aliased retry and source wedge exclusion. Owning Bazel targets
pass in `solid18-solid24-owning-bazel-build-1`. Initial native-build attempts
with an omitted precision include and an unnecessary undeclared wrapper local,
and the initial cross-unit comparison failure, are retained in earlier reports.
The independent source review found no production blocker; its suggested
small-face test is included. No vehicle owner or force loop consumes this
reference yet.

```sh
cmake -S lib_utest/qualification/solid24_reference -B BUILD_DIR \
  -DTL_SOLID24_REFERENCE_NATIVE=ON -DTL_SOLID24_REFERENCE_CUDA=ON \
  -DCMAKE_Fortran_COMPILER=/home/jsonzhou/Desktop/chrono-work/crash-work/tools/gfortran-11.4.0/gfortran-local \
  -DCMAKE_CUDA_COMPILER=/usr/local/cuda/bin/nvcc -DCMAKE_CUDA_ARCHITECTURES=120
cmake --build BUILD_DIR --parallel 4
ctest --test-dir BUILD_DIR --output-on-failure
```

Use the workspace's serialized resource guard for builds and CUDA execution.
