# Complete QBAT host binding

`ShellBatchBinding::InitializeFormulations` adds an explicit QBAT family to the
existing immutable global collection. It calls the qualified QBAT reference
producer and retains that typed reference, its complete input/options and
CNDLENI coefficient values. It does not create QBAT force histories, resident
state, a publication participant, a contact surface or source runtime admission.

```cpp
tl::fea::ShellFormulationCollectionInput input{
    {qeph_rows, t3_rows, qeph_count, t3_count, global_node_count},
    qbat_rows, qbat_count};
tl::fea::ShellBatchBinding binding;
const auto report = binding.InitializeFormulations(
    input, tl::fea::ShellHostBindingLimits::Vehicle());
```

The named entry requires a nonempty QBAT family. The legacy input structures,
pair/collection initializers and their version-3/version-4 inventory words are
unchanged. The new version-5 in-process inventory has all three family counts,
explicit native formulation words, original EIDs, ordered global indices/NIDs,
position/material/placement bits, and all thirteen integer and four binary64
QBAT options plus initial A11. It is an exact identity vector, not a persisted
file schema or a source-file authentication claim.

Each distinct source parent contributes once to the authoritative node and
collection M/J in QEPH, T3, QBAT parent/local order. Native TOTAL J remains
separate from physical/added diagnostic partitions. `qbat_totals()` is an
attribution subtotal already included in those coefficients. TYPE25 host mass
composition retains the complete inventory and adds its endpoint coefficients
once afterward. Existing catalogs, Q/T joined startup and common publication
explicitly reject the new family until a complete QBAT participant exists.

Equal physical connectivity is legitimate for distinct layers: the original
midlayer and both glass layers share node sets. Only duplicate source EIDs,
invalid within-cell repetition, inconsistent shared NIDs/position bits, missing
global coverage or rejected native reference/coefficient values fail. No
topology deduplication, invented attachment or per-material batch is introduced.

The optional retained QBAT parent array uses zero inline entries. Measured
binary64 x86-64 sizes: binding handle 178376→178448 B (+72 B), QBAT reference
832 B, retained QBAT parent 872 B, native global node 64 B. There is no new
default allocation. Existing limits remain combined 524288 parents/nodes,
1 GiB owned payload and 64 MiB startup indexes/seen scratch, with explicit
Vehicle limits needed above the old capacity. Exact array extents and shared
control reservations enter preflight before allocation/borrowed reads.
Count bounds do not guarantee admission under a smaller byte cap. Copies share
immutable owned backing; all failed initialization preserves the destination.

## Qualification and provenance

New host tests cover three layers sharing all four nodes, once-only native
partitions and independent rectangular physical mass, TYPE25 composition,
complete identity/placement/signed-zero options, count/byte/extent rejection,
late native/identity failure and retry, individually finite parents whose
combined mass overflows, copied-input lifetime, and closed old catalogs.

Two root-scheduled original-source tests use the existing authenticated
`../qbat/source_fixture/YarisQbatSourceFixture.h` and manifest in place:
4250 QBAT quads, 4384 nodes, and the separately retained EID2357656 T3.
Original inputs are thickness .0005 m, rho1000 kg/m3, E250e6 Pa and nu.35;
only virgin geometric coefficient inputs are used. A separate native test
compares every source QBAT reference/coefficient packet, all three original T3
nodal M/J and the ordered complete node/total ledger against existing independent
native QBAT/T3 reference owners. No new Fortran donor/extract is introduced.
Source pin remains `a62b27e6baa555d222a580d6218867d0be4d70b5`; original fixture
header SHA256 is `87c3902373d3a5e9e27c09522ad9adc5f4c6e6822eb2f64aa613d82bc1638c98`.

The optional CUDA-compiled rejection test exercises actual Q/T joined entry
points, with and without TYPE25 mass. It expects rejection before owner/device
allocation; it is not QBAT device execution or a trajectory qualification.

## Owning commands

Host-only:

```sh
cmake -S lib_utest/qualification/qbat_binding -B BUILD -DCMAKE_BUILD_TYPE=Release
cmake --build BUILD --target qbat_binding_host_test shell_batch_binding_check shell_plasticity_binding_check host_shell_collection_check -j1
ctest --test-dir BUILD --output-on-failure
```

Root native/source/compiled-participant gate adds
`-DQBAT_BINDING_SOURCE_CHECKS=ON -DQBAT_BINDING_NATIVE_CHECKS=ON
-DQBAT_BINDING_CUDA_CHECKS=ON -DCMAKE_CUDA_ARCHITECTURES=120` and the existing
explicit GNU Fortran compiler path. Additional targets:
`qbat_binding_original_test`, `qbat_binding_native_test`,
`qbat_binding_closed_participants_test`. New CTest names share prefix
`qbat_binding_`; affected native geometry/startup groups are also registered.
Bazel owns `:qbat_binding_host_check`, `:qbat_binding_original_check` and
`:qbat_binding_closed_participants_check` in this package. Native Fortran is
owned by the explicit CMake gate.

Author evidence: eight new and 51 existing host functions pass (four CTest
groups); eight source/native/participant C++ syntax units pass; existing T3
startup56/force67 and QBAT/fixture source verification pass. Largest guarded
build sample 402743296 B, one CPU/512 MiB. Full original-source execution,
native, NVCC and owning Bazel execution remain root qualification at handoff.

Root qualification at TL `3ab3ad8`: all63 numerical functions pass in
`qbat-binding-root-tests-1`, plus original fixture identity. This includes
the8 new host functions,51 unchanged host regressions, both complete original
4,250-quad/one-triangle tests, independent native coefficient comparison for
every original element/node, and compiled old-participant rejection. The
complete binding owns5,729,440 B and needs428,696 B startup scratch; the exact
budget and last-parent rejection/retry pass. Native TOTAL J is compared as
its own coefficient. Independent review found no blocker. The NVCC/native
build passed in119.145 s with2,617,630,720 B sampled RSS under8 affinity CPUs
and4 workers. The compiled rejection test does not execute QBAT on a GPU;
resident QBAT state/publication remains the next formulation increment.
All three owning Bazel targets also build (`qbat-binding-bazel-build-1`),
45.731 s and640,643,072 B sampled RSS with the same bounded build policy.
