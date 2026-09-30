# Extended immutable solid model

`solids::Model` retains prepared reference/material values for the three existing
families followed by `solid18_law44` and `solid18_law90`. The old aggregate input
prefix and original family order remain unchanged. Callers must select
`ModelProfile::ExtendedLaw44Law90` and supply at least one new parent; the default
profile rejects either new pointer/count span. This is immutable model scope,
not an import policy or new resident execution.

One MID index detects cross-law collisions and changed repeated declarations.
LAW36, shared LAW42, LAW44 and LAW90 have typed catalogs. All curve x/y values
are copied into one bounded arena and rebound through the existing preparation
functions. LAW44 preserves native working-unit provenance and all derived values.
LAW90 reconstructs the already resolved reader state, re-prepares against owned
curves, and requires every reader/updated scalar and curve bit to match. It
requires PM1=PM89=reference density; it does not manufacture a TIME0 history or
interpret raw MAT057 defaults. The pending original HU/SDI import decision is a
separate source-reader qualification.

`SolidNodeContributions` provides the exact domain/source-slot map and native
reference masses. Both occurrences of repeated rear H8 slots remain present;
true six-node S6Z tails remain `SIZE_MAX`. Rotation coefficients remain zero.
The material association is not a substitute for source EID/PID/MID identity.
Model copies share immutable backing; all borrowed input arrays can expire after
successful initialization. A late material/reference rejection publishes no
model and can be retried.

Limits stay at 16384 parents, 1024 materials, 1048576 owned curve points,
524288 domain nodes and 256 MiB complete startup. Actual typed headers, parent
and catalog arenas, double curve arrays, reference scratch, MID index, shared
control reserve and retained domain/coefficient storage are charged through
existing accounting. The exact cap test rejects one byte below the forecast.

The existing three-family resident `Plan` now explicitly rejects the extended
profile before layout, forecast publication, host staging or CUDA allocation.
The root-only CUDA test requires zero allocations on rejection and a successful
legacy retry. No resident Traits, element arithmetic, clock or ledger policy is
changed by this slice; the V4 coefficient ledger is independently owned.

## Owning gates

Default CMake targets `extended_solid_model_host` and
`extended_solid_model_legacy` run 11 new and 7 retained host functions. The
existing LAW44 and LAW90 preparation/source verifiers are reused directly; no
copy of a native oracle or source fixture is introduced.

Root options:

- `TL_EXTENDED_SOLID_MODEL_CUDA=ON`: actual `InitializeJoined` rejection/retry.
- `TL_EXTENDED_SOLID_MODEL_ORIGINAL=ON`: existing 2412 original cells plus rear
  306 and radiator 1345, totaling 4063 in one reordered union domain. Requires
  `REAR18_SOURCE_FIXTURE` and `TL_LAW90_RADIATOR_FIXTURE`. The existing authenticated
  readers generate only test headers. The gate compares typed geometry,
  material/curve values and all eight source mass slots, including 109 collapsed
  rear cells. Radiator geometry is original; material inputs use the already
  qualified positive-Hys prepared profile, without claiming raw HU import closure.

The original gate excludes the 837 separately admitted rubber extension; its
application source adapter/full4900 composition is a later gate. It does not
claim a full physical domain, owner, force recurrence or completed source closure.

Example root-only configuration (apply the workstation resource guard):

```sh
cmake -S lib_utest/qualification/extended_solid_model -B BUILD_DIR \
  -DTL_EXTENDED_SOLID_MODEL_CUDA=ON -DCMAKE_CUDA_ARCHITECTURES=120 \
  -DTL_EXTENDED_SOLID_MODEL_ORIGINAL=ON \
  -DREAR18_SOURCE_FIXTURE=/home/jsonzhou/Desktop/chrono-work/crash-work/reports/yaris-rear-metal-geometry-1 \
  -DTL_LAW90_RADIATOR_FIXTURE=/home/jsonzhou/Desktop/chrono-work/crash-work/reports/yaris-radiator-geometry-1
cmake --build BUILD_DIR --parallel 1
ctest --test-dir BUILD_DIR --output-on-failure
```

Bazel owning targets: `//lib_src/elements/solids:model`,
`//lib_src/elements/solids/resident:values`,
`//lib_utest/qualification/extended_solid_model:host`, and
`//lib_utest/qualification/extended_solid_model:resident_guard`.
Only fixture visibility is extended in `solid_model/BUILD.bazel`.
