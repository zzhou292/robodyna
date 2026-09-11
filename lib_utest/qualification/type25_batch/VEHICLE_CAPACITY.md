# Explicit vehicle TYPE25 capacity

The existing linear TYPE25 formulation now has an explicit capacity profile.
`Type25Capacity.h` owns all hard profile bounds. Legacy calls keep their old
1,024-connection/2,048-node domain, default budgets, arena layout, validation
order and numeric paths. Existing positional aggregates remain valid: model and
combined-mass limits append a profile field; BatchConfig gains no data member.

| Domain | Vehicle limit |
|---|---:|
| Connections | 4,096 |
| Properties | 64, unchanged |
| Global owner nodes | 524,288 |
| Model host payload including startup indexes | 16 MiB |
| Resident device arena | 32 MiB |
| Combined mass / batch host payload | 2 GiB each |

The actual literal source census is 2,828 SPOTWELD_ID records: 2,725 internal
and 103 crossing retained-shell coverage, with 5,656 unique endpoints. Optional
SPOTWELD_ID failure parameters are blank in these original records. The scope
report is `crash-work/reports/yaris-full-shell-scope-10.json`, SHA-256
`fdb51869dfd3f4de265bb4494a2d0f904c5c466bf9962b71ce19e9098a761ff0`.
Its 103 nonretained endpoints remain source-coverage obligations. Larger storage
does not admit those endpoints, infer beam behavior, release ties, or assign
auxiliary mass. This profile changes capacity only for the already qualified
spring formulation.

## Explicit API

```cpp
type25::ModelInput input = /* complete resolved source declaration */;
input.limits = type25::ModelLimits::Vehicle();
model.Initialize(input);
mass.Initialize(shell_binding, model, NodalMassLimits::Vehicle());

auto config = type25::BatchConfig::Vehicle();
config.owner = owner.accepted();
config.element_count = model.connection_count();
config.configuration_id = configuration_id;
config.qualification_id = qualification_id;
batch.InitializeJoined(config, model, mass, type25::CapacityProfile::Vehicle);
```

The static BatchConfig factory selects requested bounds, not admission. The
three-argument InitializeJoined overload and five-argument MakeLayout retain
Legacy admission. Explicit Vehicle calls must use the matching profile argument.
Byte caps remain independent from count caps and are checked before allocations.

Vehicle model startup uses the existing SourceIdentityIndex utility for source
connection/property IDs and both endpoint identity directions. First prior
occurrences preserve the original nested-loop error precedence; all reference,
history and endpoint coefficient production retains its original source order.
Temporary indexes are included in startup payload and released after startup.

Combined mass still accumulates shell contributions, then each connection and
endpoint, without rebuilding native shell J. Vehicle accounting counts embedded
handles through their actual owner and shared backing once. Batch discounts its
separate source-model payload only when SharesConnectorStorage proves the same
immutable backing. Equivalent independently created models are charged twice.
Legacy conservative byte accounting remains unchanged.

The unchanged resident layout is 512 header bytes, 208/property,
1,004/connection and 40/global node on this supported ABI. With all 64 property
slots, 2,828 connections/359,785 nodes require 17,244,536 bytes; the maximum
4,096/524,288 profile requires 25,097,728 bytes. The existing single arena,
two history/result slabs, accepted selector, force scatter, CUDA launch order,
candidate reduction, readback and publication stay unchanged. The serialized
full-node validation and source-ordered connector scatter are still potential
performance costs; no kernel optimization is claimed here.

## Qualification

Author checks under one CPU/512 MiB: ten model/startup functions pass (three
new, seven old); the new exact-layout/tail/cap function passes; all host
production and test seams pass syntax. Evidence is
`/tmp/type25-vehicle-{model,layout}-author-check.xml`. No native or CUDA build/run
was performed by the author.

Root-scheduled new gates include source-sized complete shell-derived M/J at
359,785 nodes/2,828 connectors, storage identity versus semantic equality,
exact host budget and late source-coordinate retry. The actual CUDA fixture
uses 4,096 connectors and a 524,288-node owner, with the final endpoint unique
to the last connector. It checks nonzero force/couple against the host value
operation using actual prepared fields, a late nonfinite candidate failure,
unchanged accepted cache/owner, exact retry, complete copy bounds, stable
allocations, independent-model exact-cap accounting, and a completed private
device-copy failure that cannot publish output. These uniform capacity fixtures
are not original vehicle trajectories or load-path admission.

Owning commands (root applies the workstation guard):

```sh
cmake -S lib_utest/qualification/type25 -B MODEL_BUILD -DTYPE25_NATIVE_CHECKS=ON
cmake --build MODEL_BUILD --parallel 1
ctest --test-dir MODEL_BUILD --output-on-failure -j1

cmake -S lib_utest/qualification/nodal_mass -B MASS_BUILD -DTL_NODAL_MASS_VEHICLE_CHECKS=ON
cmake --build MASS_BUILD --parallel 1 --target nodal_mass_binding_check nodal_mass_vehicle_check
ctest --test-dir MASS_BUILD -R 'nodal_mass_(binding|vehicle)' --output-on-failure -j1

cmake -S lib_utest/qualification/type25_batch -B BATCH_BUILD \
  -DTYPE25_BATCH_CUDA_CHECKS=ON -DTYPE25_VEHICLE_CUDA_CHECKS=ON \
  -DCMAKE_CUDA_ARCHITECTURES=120
cmake --build BATCH_BUILD --parallel 1
ctest --test-dir BATCH_BUILD --output-on-failure -j1
```

Corresponding Bazel owners are type25:type25_host_check,
nodal_mass:nodal_mass_vehicle_check and type25_batch:type25_vehicle_cuda_check
under `//lib_utest/qualification/`; the old layout/CUDA targets remain present.
