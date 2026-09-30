# Mapped QBAT preparation and native CIN stiffness

This qualifier covers the explicit `InitializeMapped` profile: a complete
prepared shell physical binding, the actual fresh CIN owner, exact initial raw
M/J and coordinates, and token-aware accepted force/stiffness contribution.
The existing shell-domain APIs remain separate. No family owns a commit or a
second nodal clock. Common mapped publication is a subsequent coordinator gate.

The four new host functions check scrambled physical indices and complete
source records, authentic PART M/J zero and ordinary solid J zero, exact host
preflight limits, retained producer aliases (including independently equal
domain backing), and staged native stiffness scatter. The owning executable
also runs five unchanged QBAT resident host functions.

Two independent native functions compare virgin coefficients and 160-step
four-point force/history trajectories, including removal, against the existing
native QBAT caller and the complete native CUPDTN3. `NativeInitial.F` calls the
shared current geometry packet and exact selected CNCOEF3/CNDT3 fragments; it
does not advance material history. CNDT3's selected NODADT0 coefficient is
STIR=0, distinct from initial CINMAS. The existing scatter wrapper sets NODADT1
in its context, but complete CUPDTN3 does not branch on it: this gate supplies
the independently selected CNDT3 STI/STIR and the actual two diagonal factors.
Its OFF bookkeeping is active; the QBAT caller independently supplies final
OFF-masked current forces and stiffness on removal. No donor is copied or
modified here; `native/source-manifest.json` binds the existing owners and the
small packet adapter.

Four CUDA functions exercise the actual PART + plain + ordinary + CIN owner:
foreign coherent raw mass, late source coordinate, wrong roster and absent/fixed
rotation roles reject before resident allocation; valid PART/CIN inverse zero
is admitted. The tests check actual CIN stiffness destinations, stale-token
rejection, immutable output aliases, a late prepared geometry fault, complete
accepted four-point preservation and retry. Disjoint synthetic CIN patches use
explicit prescribed test activity/stiffness/load, so this is preparation
evidence rather than a full participant roster. The successful candidate stays
uncommitted for the common coordinator. Readback rejection preserves caller
bytes; successful history comparisons use named fields rather than padding.

The optional original host function reuses the authenticated 4,250-quad and
one-T3 source/catalog fixtures without duplicating their geometry header. It
reverses all 4,384 shell-node indices and adds one explicitly synthetic point-mass
node. All source EIDs, local slots, global coefficients and four virgin points
are checked. It grants no CIN, full-source or runtime admission by itself.

Root-owned native/CUDA execution:

```sh
cmake -S lib_utest/qualification/qbat_mapped -B <build> \
  -DCMAKE_BUILD_TYPE=Release -DQBAT_MAPPED_NATIVE=ON \
  -DQBAT_MAPPED_CUDA=ON -DQBAT_MAPPED_ORIGINAL=ON \
  -DCMAKE_Fortran_COMPILER=<qualified-gfortran> -DCMAKE_CUDA_ARCHITECTURES=120
cmake --build <build> --target qbat_mapped_host_test qbat_mapped_native_test \
  qbat_mapped_cuda_test qbat_mapped_original_test --parallel <guarded-count>
ctest --test-dir <build> -R '^qbat_mapped_' --output-on-failure
```

Expected owning entries: host (9 functions), native (2), CUDA (4), original (1)
and native/source identity. Author evidence is limited to the nine host
functions, C++ syntax and source identities under 1 CPU/512 MiB. Native,
Fortran, NVCC and actual GPU execution remain root-owned until reported.

Bazel ownership targets are `//lib_src/elements:shell_physical_owner`,
`//lib_src/elements/qbat:batch` and this package's `qbat_mapped_host_check`,
`qbat_mapped_original_check`, `qbat_mapped_cuda_check`. Affected legacy runtime
gates are QBAT resident/formulation publication, CIN and rigid owner; material
and force arithmetic has not changed.
