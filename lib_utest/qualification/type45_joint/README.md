# TYPE45 joint qualification

Five small host functions qualify the authored value contracts. Four independent
native functions, one original 44-geometry function and two device-owned CUDA
functions are authored for the root owning gate; author checks do not claim
Fortran, native numerical or GPU execution.

The native oracle compiles complete RINI45/RINI45_RB/RSKEW33/RUSER33/RDTIME33/
RCUM33 bodies and their contained numerical helpers. `automatic.inc` is the
unchanged contiguous JOINT_BLOCK_STIFFNESS:134–280 per-joint block; its ALPHA
assignment is also copied directly. Its effective DTC is supplied, after the
common owner's selection/clamping stage. No native force, frame, derivative,
history or stiffness is seeded from the C++ implementation.

`Context.F90` supplies only fixed dimensions, named property slots, message
counts and the unused sensor record shape. Scalar property slots represent
already decoded supplied inputs, including the native blocked-DOF critical
damping masks. Optional curves, sensors, stops, external skews and automatic
mass scaling are absent; unsupported callback use terminates the fixture.
Complete donor bytes, SHA256, Git blob and license hashes are checked on
configure/build/test. Context headers use unique common blocks; one-element
leading dimensions are explicit in the Fortran interface module and C ABI.

The native startup registry has one matching zero-extra IXR_KJ row. This makes
GET_SKEW45's otherwise uninitialized `ID_KJ` local defined through its original
matching loop. RINI45_RB has the same lookup. This is declared supplied decoded
context, not a claim about an original exported registry. Native body records
provide principal inertia components, while main-node IN remains an independent
input. The completed native startup state is retained for every subsequent call.

Both native units are exercised. Original geometry comes from the authenticated
canonical SI node arrays and complete source key; original N1–N5, raw option
blanks and the declared 38-retained/6-boundary disposition remain visible. These
44 tests use supplied body coefficients/property values. They do not resolve
the original SDI free-DOF/default export or authenticate actual owner bodies.

The CUDA tests prepare references and advance their own private histories over
32 positive intervals for all three kinds/both units, including zero main-node J.
The independent host native oracle consumes the exact recorded device packets.
A late finite-input overflow must preserve every named state/output value before
an identical retry, without advancing the accepted history.

Root owning configuration (choose a separate build directory):

```sh
cmake -S lib_utest/qualification/type45_joint -B BUILD -G 'Unix Makefiles' \
  -DCMAKE_BUILD_TYPE=Release -DTL_TYPE45_NATIVE=ON -DTL_TYPE45_CUDA=ON \
  -DCMAKE_CUDA_ARCHITECTURES=120 -DTL_TYPE45_ORIGINAL=ON \
  -DTL_YARIS_ASSETS=/home/jsonzhou/Desktop/chrono-work/crash-work/assets/yaris-vehicle
cmake --build BUILD -j2
ctest --test-dir BUILD --output-on-failure
```

Use the shared workstation guard for those native/GPU commands. Owning Bazel
targets are `//lib_src/elements/type45:values` and
`//lib_utest/qualification/type45_joint:host_check`.
