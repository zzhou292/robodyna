# TYPE45 TT0 constructor

The first selected physical profile has zero original Kn and optional free K/C.
`PrepareVirgin` creates its pre-automatic force/stiffness cache and native
initial history values without choosing dt or creating an automatic Reference.
All three released-DOF masks stay in the separately qualified native recurrence.

Native `StartupPhase` calls complete RINI45 and RINI45_RB with automatic
assignment deferred. The constructor then calls complete RSKEW33, RUSER33,
RDTIME33 and RCUM33 at TT0/DT1=0/NCYCLE0. This mirrors the selected caller
before RESOL's later automatic UVAR assignment. `RUSER33` itself handles DT1=0
via its EP30 divisor. No positive-step force call is used as a substitute.

The existing public `type45_native_startup` and `type45_native_step` ABIs and
default behavior remain unchanged. Small shared internal phase functions add
the constructor's exact zero-time branch; the ordinary native step still
rejects dt0/cycle0. Complete pinned donor bytes and all numerical statements
are unchanged. Constructor output stages through local buffers and publishes
only after both native phases succeed.

Two host tests check the zero pre-automatic cache, nonzero post-automatic
stiffness distinction, unchanged positive-step guard and late failure/retry.
Two authored native functions cover three kinds, both working-unit profiles,
offset rigid endpoints, nonzero spin with DT1=0, all history fields, cache
forces/couples/STI/STIR and ordinary native guard/atomic retry. Root owns native
execution and the existing native/CUDA regression after shared wrapper changes.
Author `type45-virgin-build-1/tests-1` pass both host functions;
`type45-constructor-syntax-2` and `type45-constructor-source-1` pass the C++
interface check and complete donor identity preparation. No author native run.

```
cmake -S lib_utest/qualification/type45_resident -B BUILD \
  -DCMAKE_BUILD_TYPE=Release -DTL_TYPE45_RESIDENT_NATIVE=ON \
  -DTL_TYPE45_CUDA=ON
cmake --build BUILD -j2
ctest --test-dir BUILD --output-on-failure
```

Actual owner token/main-coefficient query and Batch publication are subsequent
resident layers. This constructor gate claims no source coverage or live owner.
