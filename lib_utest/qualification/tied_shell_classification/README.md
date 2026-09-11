# Supplied-context TYPE2 classification

This host startup value block follows pinned ITAGSL2 and the six-direction
KINSET branch used by ITAGSL2/CHECKRBY. It does not import source constraints,
perform search, claim an original CIN count, run KINCHK, bind an owner or add
mass/forces. The app must supply the complete post-search interface roles and
pre-classification native condition context with independently authenticated
source/event associations.

`Classify` preserves the ordered global marking passes, all five IKINE fields,
and the native mutation of `ITF(IKINE(node))`. It reconstructs fresh KININI lookup
tables per invocation. Twelve condition kinds select penalty; IWL alone does
not. Repeated selected slaves mark all their occurrences before the final
registration pass. Source interface IDs and ordered slave-node indices are
retained in the immutable result; nonselected interfaces are distinguished from
CIN. The result phase is `InterfaceTaggedBeforeKinChk`.

`RegisterRigidMembers` is one CHECKRBY registration scope after caller-supplied
hierarchy resolution. It resets one shared private scratch array, visits each
group/member and six directions in supplied order, and registers kind 8 or 128
according to the supplied global IKREM when IDDLEVEL=0. A nonzero IDDLEVEL skips
this native registration. It does not resolve original rigid flags, generated
primary IDs, source group hierarchy, wall exclusions or later conflict handling.

The five supplied fields are condition code, translation/skew, rotation/skew,
same-kind duplicate conditions and different-kind incompatible conditions.
Lookup-table codes are bounded 0:8191; packed directions have native low digit
0:7 and nonnegative skew. Counts, spans, cumulative membership and host bytes
are preflighted before payload reads. A tiny byte limit also rejects before
nested role metadata reads. The byte forecast includes the old result, staged
owned values, scratch tags/registration and temporary source-ID index. It is a
payload limit, excluding allocator bookkeeping and caller-owned borrowed input.
Default limits are 1,048,576 nodes, 4,096 interfaces, 4,194,304 occurrences and 256 MiB.
An additional native counter bound protects five-block indexing and accumulated
six-direction warning counts if callers raise their configured limits.

Results publish by one final nonthrowing shared-handle swap. Invalid final
records, allocation failure and budget rejection retain the prior result. Input
views may borrow an old result; reads finish before publication. No per-step
allocation, device code, state selector or mechanics clock is introduced.

## Independent native oracle

Pin: `a62b27e6baa555d222a580d6218867d0be4d70b5`.
The oracle compiles complete unchanged ITAGSL2, KININI and KINSET. The complete
INTAB function is extracted from i24tools.F:95–118. The conditional CHECKRBY
registration loop is extracted exactly from checkrby.F:134–150 and placed inside
an explicit supplied-group/member loop with the same scratch lifetime. It does
not execute CHECKRBY's hierarchy, duplicate-report sorting or source reader.

The six retained complete donor files and both extraction boundaries are
verified by the existing shared source verifier with size/SHA256/Git blob
identities. The test context replaces only diagnostics, allocation and the
accessed INTBUF members/dimensions. It keeps the complete original KINCOD common
block; tables and counters reset for every call. Entry interfaces and independent
packet dimension/index checks are explicit. The C++ oracle does not call the
production classifier or its validation/registration helpers.

Comparisons cover every nodal block, every IRUPT row and all 8,192 ITF entries,
plus native KWARN and the 1179 penalty-warning count. Fixtures exercise each
penalty condition, wall/skew interaction, source-order global ITF interference,
same/cross-interface duplicates, every selected main-role level, section/cyclic
and RBE roles, both tetra corner passes, rigid-member duplicates and scratch
resets, bad final fields and exact retry. Native integer equality is required;
there are no numerical tolerances or production-derived oracle seeds.

## Owning gates

Author host-only configuration (one worker and a 512 MiB guard outside CMake):

```sh
cmake -S lib_utest/qualification/tied_shell_classification -B /tmp/tied-classification-host -DTL_TIED_CLASSIFICATION_NATIVE=OFF
cmake --build /tmp/tied-classification-host -j1
ctest --test-dir /tmp/tied-classification-host --output-on-failure
```

Root native gate uses the same commands with a separate build directory and
`-DTL_TIED_CLASSIFICATION_NATIVE=ON`. Targets are
`tied_classification_host_test` and `tied_classification_native_test`; CTest names
are `tied_classification_values`, `tied_classification_source_identity` and
`tied_classification_native`. The native target enables runtime bounds checking.
Bazel owns production `//lib_src/constraints/tied_shell:tied_shell_classification`
and host `//lib_utest/qualification/tied_shell_classification:tied_classification_values_check`.

At author handoff, nine host functions and source identity passed; both native
C++ test units passed syntax checking. Fortran/native execution belongs to the
root qualification gate. No original-source classification or GPU run is claimed.
