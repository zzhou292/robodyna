# True one-point T3 resident qualification

This increment adds `ShellSectionFormulation::OneThicknessPoint` and the resolved
`ShellSectionLaw::Law44Nip1` role to the existing complete section catalog and T3
participant. The historical material tags retain their old public values and
meaning; one material may feed both one- and three-point sections. Legacy
initializers still require LAW44 NIP3. One-point admission requires centered T3,
LAW44 and complete positive constant D1 failure declarations. It does not admit
the full original midlayer PID, QBAT quads, contact removal, ties or a new owner.

The optional state is exactly 224 B per family row: one genuine material packet
(current/saved/failure/thickness/work) and cumulative WPLA. Two arrays follow the
existing T3 accepted selector. The allocation is 16+448*N B, plus one N-row host
readback array and explicitly forecast startup staging. No selector, clock,
per-step allocation or additional shell history is introduced. Existing mixed
and failure arenas retain their layouts; inactive NIP3 capacity for these rows
is never interpreted or advanced as point history. The public layered union is
still 240 B. HostStorage gains one optional pointer; its `sizeof` is included in
the existing host forecast. Default device allocation counts are unchanged.
All optional/base arenas must fit the existing 2 GiB vehicle family cap; the
524288 parent ceiling does not guarantee every complete layout fits.

`Copy*LayeredSectionHistory` returns `.one_point()` for NIP1, and null for both
`.plastic()` and `.elastic()`. The full family is staged and checked for source
role, finite values, flag encodings, saved/current/failure relations, native
point domain and the actual shell slab's reference/stamp/thickness/cache before
publication. Failed prepared reads discard the candidate; CUDA failures poison
the participant. Old three-point `Copy*FailureHistory` rejects a family with
NIP1. Padding and inactive union bytes are not a value or serialization contract.

The host gate covers role/material/family separation, complete failure policy,
exact bytes/caps, absence of NIP3 history access, 96 adapter intervals, late
rejection/retry and original triangle collection setup. Existing catalog tests
remain in the owning target. The actual fixture places original EID2357656,
NIDs2300357/2300138/2300139 with original SI coordinates and material values
inside three explicitly synthetic neighboring parents. This is prescribed owner
history qualification, not a vehicle trajectory or new source adapter. D1=2.5
is used in the original-value history test; D1=1e-6 is a separate failure timing
control. The native startup gate reuses the independent existing C3EVEC3/
C3INMAS/C3DERII oracle: its selected ordinary mass/inertia and zero-derivative
branches are NPT-independent. Starter STI/SSP are not outputs of this gate.

Five actual CUDA functions cover native force/history agreement, once-only
combined M/J, unload/reload, removal/current work, late geometric failure and
retry, nonfinite/invalid-flag/finite-shell-mismatch readbacks, completed-copy
failure, stale/alias/cap rejection and legacy allocation preservation. Their
native state advances independently only after common publication. Readback
fault injection is test-only and forwards/completes the real copy first.

Author evidence: catalog 21 functions PASS (3 new), resident host 4 PASS; six
production storage/readback C++ units and native startup test syntax PASS.
One layout test initially omitted the base shell arena from its aggregate cap
expectation; corrected test passes, with the initial failure report retained.
Clang14 host-only CUDA syntax is unavailable with installed CUDA13 headers;
no actual NVCC/native/GPU execution is claimed by the author.

Root owning commands (guarded serial execution; explicit GNU Fortran toolchain):

```sh
cmake -S lib_utest/qualification/t3_one_point_resident -B BUILD \
  -DCMAKE_BUILD_TYPE=Release -DT3_ONE_POINT_RESIDENT_NATIVE=ON \
  -DT3_ONE_POINT_RESIDENT_CUDA=ON -DCMAKE_CUDA_ARCHITECTURES=120 \
  -DCMAKE_Fortran_COMPILER=/ABS/PINNED/gfortran
cmake --build BUILD --target shell_plasticity_binding_check \
  t3_one_point_resident_host_test t3_one_point_startup_native_test \
  t3_one_point_resident_cuda_test -j1
ctest --test-dir BUILD -R '^shell_plasticity_binding_check$|^t3_one_point_(resident_host|startup_native|resident_cuda|resident_source_identity)$' --output-on-failure
bazel test //lib_utest/qualification/plasticity_binding:shell_plasticity_binding_check \
  //lib_utest/qualification/t3_one_point_resident:t3_one_point_resident_host_check
```

Production Bazel ownership remains `//lib_src/elements/t3:batch` and the existing
shared storage target. `t3_one_point/OnePointNative.cmake` factors only the already
qualified native library construction for reuse; it changes no Fortran wrapper
or donor. Source hashes retain all old native records and append the reviewed
T3 build registration. The original source fixture is reused through its owning
QBAT test-only target without duplication.

Root owning gate passes **31 functions** (21 catalog, four host, one original
triangle startup native and five actual CUDA) plus source identity. Reports:
`t3-one-point-resident-root-{configure,build,tests}-1`, XML functions1.
The native startup comparison includes every nodal/element TOTAL J, physical J,
added J and mass component; the CUDA history remains inside the existing common
publication. Build used eight affinity CPUs/four workers and sampled
2,731,995,136 B process-tree RSS. Tests used two CPUs/2 GiB; sampled RSS
169,455,616 B. These are prescribed-state integration checks, not a new impact
trajectory. Affected old resident/source-flight regressions are tracked separately.
Those affected gates now pass all50 functions:11 resident TAB1,13 resident
constant failure,18 mixed/resident plasticity and8 actual-source CUDA flight.
Reports `one-point-resident-{tab1,constant,mixed,source-flight}-tests-1` retain
their separate XML evidence. Original149/631-parent fixtures remain covered.
