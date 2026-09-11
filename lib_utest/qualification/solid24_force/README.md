# Mapped LAW42 HEPH force value gate

This gate advances one HEPH24 element from prescribed source-slot positions and
velocities. It reuses the immutable total-strain reference, qualified LAW42
point/caller, frame transform, characteristic length and physical-mode helpers.
It does not admit a vehicle source profile, collection, nodal owner or contact
participant. Native and CUDA tests are authored for the root qualification lane;
a host syntax check is not evidence that either executed.

## Selected mechanics and state

The explicit mapped profile is engine JHBE24, ISMSTR10, JCVT1 (public Iframe2),
ISORTH0, ICPRE1, IINT2, JLAG1, NPT1, IMATVIS1, ISCTL0, default IDTS6=0, DN=.1,
QA=1.1 and QB=.05. The material has one alpha=2 Ogden term, no Prony terms, and
positive density. Original rubber controls use mu=24 MPa, nu=.463 and the exact
source density conversion. PM22 is the native GS=2*mu, not printed MU0. SZETFAC
scales that modulus by max(.05,ET) for ET<=1 and .2+ET otherwise. The postmaterial
sound speed supplies both artificial bulk viscosity and physical stabilization.
No source IHQ2/QM coefficient is copied into this mapped HEPH profile.

`History` owns its prepared reference/material identity, six stresses, density,
bulk pressure, one internal-energy density, twelve FHOUR values and an interval
stamp. The caller supplies the existing base time, dt and next sample index.
`EvaluateForce` validates exact reference/source/material fields and phase, stages
all values, then replaces the output once. Failed inputs, late overflow and
material cutoff leave an earlier trial unchanged. Assignment of the returned
history is the value gate's explicit acceptance; it introduces no solver clock.
The returned raw element dt is diagnostic, not a timestep admission policy.

The sequence is current cyclic frame and local velocities; initial global-JAC
material displacement gradient; current derivatives/finite-step velocity rates;
SRHO/MMAIN/MULAW and MQVISCB; SZHOUR3; SFINT3; inverse frame and original-slot
scatter. Both SZHOUR half-work terms update the same accepted EINT as material
work. `diagnostics.material` retains the material-stage values before these HG
increments; it is not a second accepted energy history.

This first force profile requires active positive geometry/material. Native
LAW42 tensile cutoff is observed and atomically rejected instead of inventing
post-cutoff deletion/history behavior; the mapped original cutoff is 1e26 Pa.
ALE, mass scaling, Prony/thermal terms, ISCTL, small-strain switching and element
removal are outside the API's selected scope.

## Units and original source

The value packet is SI. Runtime native EM20 volume/area/Jacobian denominators are
compared using that SI packet, as in the shared LAW42 caller. This does not claim
working-mm equivalence at sub-floor volumes. Reference startup separately retains
its typed metre/mm floor contract. All 1,309 original brick geometries are away
from these floors. Their immutable source slots, EIDs and source associations
come from the shared `solid_common/source_fixture` 1,504-cell inventory; wedges
are excluded by its exact topology check.

HEPH24 is an explicit demo formulation mapping. The literal original hourglass
card and converter Isolid1 choice are preserved in source evidence, not silently
relabelled as native HEPH input. Actual runtime/source admission remains separate.

## Independent native oracle

`native/source-manifest.json` binds complete official OpenRadioss donors to
`a62b27e6baa555d222a580d6218867d0be4d70b5` by path, byte count, SHA256 and Git blob.
The full `SZFORC3` is retained as caller-phase evidence. Complete SZDERI3,
SZDERITO3, SREPISO3, SORTHO3, SRROTA3, SGCOOR3, SDEFOT3, SZTORTH3, SORDEFT3,
SDLEN3/SLEN, SDEFO3, SZHOUR3/SZETFAC and SFINT3 execute independently of production.
Complete SCHKJAB3 is also linked because the unchanged SZDERI3 object contains
the unselected SZDERIT3 entry point; its call remains independently namespaced.
The immutable shared native material packet executes SRHO/MULAW/MMAIN/MQ and
complete SIGEPS42. The startup oracle supplies its own source permutation and
reference Jacobian. The family wrapper receives no production geometry, stress,
hourglass state, modulus, sound speed or force as oracle input.

Source preparation namespaces symbols/common blocks and extracts explicit dummy
interfaces with their original leading bounds. Only SZHOUR3 receives two extra
observation calls, after its EINT updates at original lines 462 and 688. They copy
work and GG/FCL and never feed a native expression. Unsupported context branches
fail loudly. The aliases at `allocbuf_auto.F:1777-1795` establish one-point global
and local EINT/stress/rho storage identity; `freform.F:875-887` establishes the
selected default IDTS6. The native point's independent material setup owns PM22.

The packet compares 187 named fields: complete accepted material/FHOUR/time,
original-slot RHS, frame/current local geometry/velocities, volume/length,
derivatives/projection/Jacobian diagonal, total gradient/rate, all 33 material
caller channels and three physical-stabilization observations. Stress fields
reuse the separately qualified material norm comparison. Other tensor/vector
groups use a relative operation-group norm; scalar channels retain per-value
bounds. No raw C++ padding participates in comparisons.

## Owning gates

Host: four functions cover persistent physical modes and one energy, force
balance, exact reference/phase rejection, finite overflow and exact retry.
Independent native: two recurrent/rejection functions plus all 1,309 originals
for three intervals. GPU: device-owned 32-interval histories on regular,
distorted and reversed geometry, late overflow/identity/cutoff rollback and exact
retry, plus all 1,309 original histories for three intervals. Histories are never
reseeded from native or host production during the GPU recurrence.

```sh
cmake -S lib_utest/qualification/solid24_force -B build/heph-force \
  -DCMAKE_BUILD_TYPE=Release -DTL_SOLID24_FORCE_NATIVE=ON \
  -DTL_SOLID24_FORCE_CUDA=ON -DCMAKE_CUDA_ARCHITECTURES=120
cmake --build build/heph-force --target \
  solid24_force_host solid24_force_native_test solid24_force_cuda -j1
ctest --test-dir build/heph-force --output-on-failure -j1
```

Run these only through the workspace's scheduled bounded native/GPU lane.
Production Bazel ownership is `//lib_src/elements/solid24:force`; the bounded
host target is `//lib_utest/qualification/solid24_force:host_check`. Two tiny
qualification-only `test_values` targets reuse existing serializers/reference
fixtures. No native/reference source enters production.

## Root qualification checkpoint

Ten numerical functions and five source identities pass in
`solid24-force-root-tests-3`: all1309 original bricks over three native/CUDA
intervals,32-step independent histories, and late rejection/retry. The owning
build gate is recorded separately. The original52-file donor set needed the
complete SCHKJAB3 dependency of the retained secondary SZDERIT3 routine,
so the final checked set contains53 files.

The native wrapper explicitly selects `INVSTR=35`, the boundary for the modern
GEO(13) CVIS property layout. Its initial zero context selected legacy PM(4),
incorrectly disabling damping and causing force/work mismatches. The fixed
wrapper and direct positive native-FCL control preserve the production
equations and all comparison tolerances. The original failed build and
numerical reports remain available.
