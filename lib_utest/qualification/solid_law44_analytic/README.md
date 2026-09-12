# Analytic 3D LAW44 and selected airbag element qualification

This adds the actual MFUNC0 branch of pinned OpenRadioss SIGEPS44 to the existing
three-dimensional LAW44 point. The table entry remains the default. Analytic
materials own only scalar declarations and require a canonical empty curve and
cursor zero. There is no synthetic table, new material family, owner or clock.

`PrepareMat024Analytic(material, E_working, SIGY_working, ETAN_working, output)`
executes the converter's `ETAN*E/(E-ETAN)` before converting A/B once to SI. It
checks the supplied E against the prepared SI declaration. The source adapter
must preserve its raw cards, working units and selected formulation separately.
`PrepareAnalytic` also admits bounded finite A>0, B>=0, n>=0 declarations and
optional positive stress/plastic limits; zero selects the actual finite reader
default. The default positive-rate, total-rate/filter and active convected
caller scope is unchanged. EPSGM is derived in native stress units and remains
distinct from EPMAX. Native yield/tangent expressions, including H=E at PLA=0,
are executed before the unchanged radial return and pressure operations.

The Model copies scalar identity and retains no curve bytes for an analytic
material. Mixed table/analytic MID collisions or changed repeated MID values
reject. Resident upload/readback preserves empty pointers and authenticates the
same hardening kind/scalars. All arena bytes use the actual extended types.
Beam18 remains a separate tabulated-only caller and must guard that scope; this
solid extension does not admit analytic beam mechanics.

The complete SIGEPS44/MSTRAIN_RATE/VINTER donors and exact reader/MULAW excerpts
are reused from `solid_law44_point/native`. New independent wrappers select
MFUNC=NVARTMP=0 and pass original scalar declarations, not production-derived
moduli or EPSGM. The existing table wrapper and complete donors are retained.

The airbag source receipt selects PID/SID/MID2000945: 80 cells, 156 nodes, and
four `[A,B,C,D,E,E,F,F]` cells with EIDs2167690,2167709,2167737,2167756. The
original blank ELFORM/global IHQ4/QH.02 resolves Isolid5. The explicit demo
selection is the already qualified Isolid18/JHBE17/2x2x2/ICPRE1/ISMSTR2/JCVT1
operator. Both dispositions are retained. No Isolid5 trajectory equivalence or
full vehicle/source closure is claimed. Reference, slot-M and constructor
checks must pass for all80 before the app can admit that named selection.

Root-only staging reuses the existing authenticated app selector, original
working-coordinate collector and binary array writer. There is no new source
card parser. The exported manifest hash is an explicit input to the build;
the generator also checks the pinned ordered80-row source hash, declarations,
all arrays, exact one-way coordinate conversion and four collapsed identities.

```sh
python3 -B lib_utest/qualification/solid_law44_analytic/source_fixture/export_fixture.py \
  --app-root /home/jsonzhou/Desktop/chrono-work/robo-dyna \
  --assets /home/jsonzhou/Desktop/chrono-work/crash-work/assets/yaris-vehicle \
  --source-member /home/jsonzhou/Desktop/chrono-work/crash-work/assets/yaris-cli-source-1/yaris-coarse-v1l.key \
  --output /home/jsonzhou/Desktop/chrono-work/crash-work/reports/yaris-airbag80-geometry-1
cmake -S lib_utest/qualification/solid_law44_analytic -B BUILD \
  -DSOLID_LAW44_ANALYTIC_NATIVE=ON -DSOLID_LAW44_ANALYTIC_CUDA=ON \
  -DAIRBAG_SOURCE_FIXTURE=/home/jsonzhou/Desktop/chrono-work/crash-work/reports/yaris-airbag80-geometry-1 \
  -DAIRBAG_SOURCE_MANIFEST_SHA256=HASH_FROM_EXPORT_RECEIPT
cmake --build BUILD --parallel 2
ctest --test-dir BUILD --output-on-failure
```

Run those commands only in the root's serialized resource guard. Author work
does not execute the native/CUDA or complete original fixture gates. The new
point-native tests cover TT0, independent accepted elastic/plastic/unload/reload
histories, both unit profiles, nonunit/zero exponent, reader limits and nextafter
neighbors. CUDA checks the same recurrence and late rejection/retry. A mixed
five-family resident test uses the existing sole-owner qualification peer,
actual ledger/PART/plain/CIN proofs and late publication rejection. It proves
the new material's resident support, not an independently published vehicle.

Owning host Bazel target: `//lib_utest/qualification/solid_law44_analytic:host`.
Root regressions: original `solid_law44_point`, `solid18_law44_startup`,
`solid18_law44_force`, `extended_solid_model`, `extended_solid_resident` and
beam18's explicit table admission. Keep their complete native/source gates.
