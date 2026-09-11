# QBAT complete host catalog qualification

Phase 1 owns declarations, immutable source/failure identity and a borrowed
scope validator. No force implementation, resident state, publication, CUDA or
source-runtime admission is added. Qualified geometry remains owned by
`qbat_binding` and its existing independent native reference fixtures.

The default host group has eight functions: same-PID/MID/SID NIP1 resolving
T3 one point and QBAT four points; coincident three-layer Q4/T3 declarations;
absent QEPH/T3; deep material/curve ownership and copied index lifetime; strict
old initializers; late source/A11/role/failure rejection and retry; count/range
preflight and exact owned/scratch budgets; complete optional TYPE25 mass scope.
Synthetic glass rows test declaration identity only and do not claim original
glass runtime admission or exercise their force law.

Author qualification (one CPU, 512 MiB):

- Eight new host functions pass, report `/tmp/qbat-catalog-tests-1.xml`.
- All 51 affected old host functions pass (21 material/section, 14 collection,
  16 geometry binding), `/tmp/qbat-catalog-legacy-tests-1.json`; default four
  CTest groups contain 59 total passing functions.
- Owning build peak sampled RSS 319217664 B; affected legacy build peak
  402948096 B. No native/Fortran/NVCC/GPU jobs by the author.
- Original test source syntax passes, 123551744 B sampled peak; actual source
  execution remains root-owned.
- T3 startup/force provenance (56/67 records), QBAT geometry/force source closure
  and original fixture identity pass unchanged, `/tmp/qbat-catalog-provenance-1.json`.

The optional original group has two functions. It reuses, without duplication,
`qbat/source_fixture/YarisQbatSourceFixture.h` and its authenticated manifest,
plus `qbat_binding/OriginalFixture.cpp`. All 4250 original PID2000524 quads and
EID2357656 (original source line 274492) enter the same catalog in original
source-line order. The 4384-node union, one MID/SID, original analytic
E=250e6 Pa, nu=.35, rho=1000 kg/m3, t=.0005 m, SIGY=10e6 Pa, ETAN=1e6 Pa,
resolved filtered C=0/P=1/Fcut=10000 and D1=2.5 retain the previously qualified
source parameter contract. Tests cover every family/source lookup, exact budgets,
the final source assignment/failure and retry. These are host identity and
composition checks, not a new independent material recurrence oracle.

Root owning commands:

```sh
cmake -S lib_utest/qualification/qbat_catalog -B BUILD \
  -DCMAKE_BUILD_TYPE=Release -DQBAT_CATALOG_SOURCE_CHECKS=ON
cmake --build BUILD --parallel 1 --target qbat_catalog_host_test \
  qbat_catalog_original_test shell_plasticity_binding_check \
  host_shell_collection_check shell_batch_binding_check
ctest --test-dir BUILD --output-on-failure --parallel 1
```

Expected: 61 numeric functions in five numeric groups plus original fixture
identity. No Fortran/CUDA compiler is needed for this phase. Explicit Bazel
owners are `//lib_utest/qualification/qbat_catalog:qbat_catalog_host_check` and
`:qbat_catalog_original_check`; production scope target is
`//lib_src/elements:shell_formulation_scope`. Root owns Bazel and the optional
actual source gate. Existing `qbat_binding` compiled participant tests continue
to prove old Q/T entry points reject QBAT before device allocation.
