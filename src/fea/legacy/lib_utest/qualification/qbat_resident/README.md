# Complete QBAT resident participant

Root qualification (2026-09-11, production `0dfb926`, fixture corrections
`8fea40c`, build dependencies `61e9180`) passes all17 new functions: five host,
two independent native and ten CUDA, including all4,250 original quads plus
one T3 on three owner intervals. All62 affected old TL/app functions also pass:
52 shell/failure/mixed, seven combined-mass/connector publication and three
actual six/seven-part wall cases. Owning QBAT and common-publication Bazel
targets compile successfully. Evidence prefixes in `crash-work/reports/` are
`qbat-resident-root-tests-2`, `qbat-affected-{resident,mass,source-wall}-tests-1`
and `qbat-domain-owning-bazel-build-3`. Failed first attempts remain recorded;
fixes affect fixtures/package dependencies, not native arithmetic or tolerances.
This qualifies the resident/common-publication slice, not full-vehicle assembly.

This gate composes the already qualified four-surface-point QBAT recurrence
with the existing nodal owner and shell publication. It adds no material,
failure, reference, mass, force, contact or clock equation. The original
midlayer fixture is 4,250 QBAT quads plus the separately qualified one-point
T3 EID 2357656. Original fixture bytes and native donors remain owned by the
existing `qbat`, `qbat_binding` and `qbat_force` qualification targets.

`qbat::Batch::InitializeFormulations` takes the complete immutable
`ShellFormulationScope`. QEPH and T3 have matching explicit initializers;
their old standalone/joined paths retain their existing admission checks.
`ShellBatchPublication::InitializeFormulations` takes the closed
`ShellFormulationParticipants{qeph,t3,qbat,connector}` aggregate. QBAT is
required; QEPH/T3 pointers are present exactly when their immutable family
counts are nonzero. TYPE25 is present exactly when the scope contains its
matching combined mass binding. This still requires complete shell coverage
of every owner node; beam endpoints, solid welds and auxiliary nodes are not
admitted here.

After a real initial assembly and caller discard, the coordinator claims all
participants. Each later attempt assembles its accepted caches (coupled mode),
uses the sole owner's seal and staggered advance, evaluates each complete
family, and passes all typed candidates to `PrepareFormulations`. The existing
`Commit` authenticates the receipt and owner token, commits the owner once,
then performs only infallible participant selector changes. Failed validation
discards trial state. CUDA failures poison the participant/coordinator before
publication. Fresh free rotational owner nodes and reference rest/common
translation use the same initial physical-source restrictions as Q/T.

Each pointer-free `BatchResult` contains 3,216 bytes of numerical history,
kinematics, current point observations, world forces/couples and native
diagnostics. It contains the four actual independent surface histories,
including saved masked stress versus current force stress. IPG4 removal and
final OFF projection come directly from the qualified pure recurrence. Native
EINT, WPLA and EVIS remain separate signed ledgers; WPLA/EVIS are not added to
EINT or advertised as an energy closure. The only kinetic sum comes from the
existing complete nodal M/J reduction. There is no fabricated NIP3 state or
per-part kinetic sum.

The one device arena uses two complete result slabs, one accepted selector,
an immutable1,088-byte (including both retained projection-metric identities) element record, shared global nodal mirrors and one
catalog curve pool. Its variable extent is `7508*parents + 56*nodes +
16*curve_points` bytes, plus the bounded header/alignment. The original
4,250/4,384 scope is below 33 MiB. A 524,288-parent maximum-count request can
exceed the explicit 2 GiB family budget and is rejected; the count ceiling
does not guarantee admission. Host preflight counts the whole startup arena,
3,216 bytes of reusable readback staging per parent, the Impl, immutable
bindings/catalog/failure and optional combined ledger. The existing backing
identity helper discounts only actual shared inventory storage. Combined
mass accounting remains conservatively inclusive. There is no per-step heap
or device allocation.

On the owning 64-bit compiler, the new optional Q/T initializer flag increases
each Q/T Impl by 8 bytes. The trailing common diagnostics add 256 bytes
(800 to 1,056); common Impl grows by 520 bytes (2,128 to 2,648). Existing host
preflight includes `sizeof(Impl)`, so unchanged byte budgets still enforce
these extents. Existing public aggregate fields retain their order. The Q/T
result/section union and legacy device layouts remain unchanged. Readbacks
validate every active named value before copying the complete requested
array. Common outputs also check every present participant's actual backing,
including independently allocated but equal catalogs. Object padding is not a serialization or equality contract.

Author evidence: five owning host functions pass in
`/tmp/qbat-resident-host-tests-3.xml`; build peak sampled RSS 362,123,264 bytes,
one CPU. Twenty host-parsable production/test units pass syntax checking in
`/tmp/qbat-resident-syntax-10.json` (225,951,744-byte peak). This does not compile
CUDA kernels or establish device/native execution. The T3 provenance checks
pass 56/67 records, preserving historical and original donor identities.

The root-owned gate includes two native tests (independent yielding,
unloading, rollback and IPG4 removal histories) and actual CUDA tests for all
four Q/T presence combinations, distinct original layers with coincident
topology, optional TYPE25 combined kinetic partitions, finite/flag/copy
failures, stale/cap/alias rejection, native four-point comparison and stable
allocations. The original-population CUDA test compares all 4,250 QBAT packets
against independently carried native state on three actual owner intervals;
its late readback fault targets parent 4,249. The copy-error hook is an explicit
test-only completed CUDA API failure simulation, not a hardware fault claim.

Root configure/build/test commands, run through the workstation's existing
serialized resource gate:

```sh
cmake -S lib_utest/qualification/qbat_resident -B /path/to/root-build \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_Fortran_COMPILER=/home/jsonzhou/Desktop/chrono-work/crash-work/tools/gfortran-11.4.0/gfortran-local \
  -DCMAKE_CUDA_COMPILER=/usr/local/cuda/bin/nvcc \
  -DCMAKE_CUDA_ARCHITECTURES=120 \
  -DQBAT_RESIDENT_NATIVE=ON -DQBAT_RESIDENT_CUDA=ON
cmake --build /path/to/root-build --parallel 1 \
  --target qbat_resident_host_test qbat_resident_native_test qbat_resident_cuda_test
ctest --test-dir /path/to/root-build --output-on-failure -R '^qbat_resident_'
```

Owning Bazel production/host targets are `//lib_src/elements/qbat:batch`,
`//lib_src/elements:shell_batch_publication` and
`//lib_utest/qualification/qbat_resident:qbat_resident_host_check`. Root also
retains the affected old Q/T mixed, one-point/failure, connector-publication
and source identity gates. A later archive adapter must explicitly publish
the real 1/3/4-point roles; the existing NIP3 component producer is unchanged.

Affected resident selectors (in their existing owning build trees):
`^mixed_shell_batch_check$`, `^mixed_layered_resident_check$`,
`^t3_one_point_resident_(host|cuda)$`, `^resident_shell_failure_(host|cuda)$`,
`^resident_shell_tab1_(host|cuda)$`, `^connector_publication$`, and
`^nodal_mass_joined$`. These are separate from the new root build above.
