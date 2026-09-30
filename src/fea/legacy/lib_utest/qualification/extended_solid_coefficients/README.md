# Extended solid coefficient snapshots

`SolidCoefficientInput` preserves its original aggregate prefix and three-family
order. The explicit `ExtendedLaw44Law90` profile appends typed prepared LAW44 H8
and LAW90 total-strain H8 references, in that order. Either new family may be
present by itself. An extension with neither new family, an unknown profile,
or new fields supplied under the original profile is rejected.

This adapter retains immutable original EID/PID/SID/MID, source NIDs, exact
source-to-domain indices and native prepared source-slot masses. It does not
retain borrowed references/materials. Collapsed H8 connectivity still has all
eight source mass slots: two equal NIDs contribute two separate native slot
masses. Native geometry permutation does not reorder these source slots. True
S6Z references retain their original six slots. All families retain exactly
zero nodal rotational inertia from the existing selected solid mass policy;
no filler mass or inertia is introduced.

The existing Map and Same functions are unchanged. All five typed pointer/count
pairs and the combined bounded count are checked before source lookup or
allocation. The startup reservation includes actual enlarged implementation
metadata, one source identity index, the full retained domain and the parent
arena. Failed final-family mapping destroys the private candidate and leaves
an empty handle available for retry. Successful handles share immutable backing;
repeat initialization cannot mutate them. Existing byte limits are unchanged.

This is coefficient/source mapping only. The ordinary ledger explicitly rejects
any extended snapshot before composition. There is no new resident, force,
material, mass floor, source-closure, rigid, DOF or owner admission. A later
explicit ledger/profile extension must provide that independent authority.

The nine new host functions cover:

- All five families in the preserved order; exact native masses, original source
  identity, repeated H8 slots and zero rotational inertia.
- Original-profile and old-ledger rejection, then successful legacy retry.
- Final-family duplicate identity, one-bit coordinate mismatch, missing NID and
  unprepared reference failures, with empty-handle retry and immutable success.
- Every family's null/count, nonnull/zero, misaligned, overflowing range and
  excessive count controls before access.
- Exact combined parent, domain-node and inclusive host-byte caps.
- Mutation and destruction of borrowed references, independent equal snapshots,
  shared copies, exact old prefix parity, and single-family/reordered domains.

The old 16-function coefficient suite runs unchanged in the same CMake project.
`verify_sources.py` pins current adapter/test files, authenticates unchanged
Map/Same bodies and the original input prefix, and runs the existing LAW44 and
LAW90 reference source verifiers. Only the two necessary fixture BUILD receipt
records change; their prior values remain in the manifests. Existing reference,
fixture and native donor bytes remain unchanged.

```sh
cmake -S lib_utest/qualification/extended_solid_coefficients -B <new-build> -DCMAKE_BUILD_TYPE=Release
cmake --build <new-build> --parallel 1
ctest --test-dir <new-build> --output-on-failure
```

Expected: 9 new + 16 unchanged host functions and the source identity CTest.
Bazel: `//lib_utest/qualification/extended_solid_coefficients:host`,
`//lib_src/assembly:solid_node_contributions`,
`//lib_src/assembly:nodal_coefficient_ledger`.

The author passed those 25 host functions and source checks in 11.281 seconds
under 1 CPU / 512 MiB, peak sampled RSS 301,764,608 bytes. No native execution,
CUDA, full-source or resident qualification was run in this lane. Root owns
native/full/owning gates and subsequent complete source/profile composition.
