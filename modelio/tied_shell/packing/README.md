# Tied-shell source packing

`TiedShellPacking` retains the immutable declaration and canonical backing.
It adds master-row/rank permutations only; existing ascending-original-NID
slave and master-node arrays supply the node order. Canonical indices address
source arrays and are never presented as original EIDs or native registered
element indices. No coordinates, shell/beam/solid geometry or source cards are
duplicated in the returned handle. Constraint evidence remains available through
the declaration, whose classification/search state remains unresolved.

The explicit `PackingPolicy` selects the pinned direct-import converter's
single additive PART clause, default source options and normal orientation,
and true Q4/T3 topology. `CREATE_SURFACE_FROM_ELEMENT` sorts native
`(n1,n2,n3,n4,family_element_index)` keys through stable `MY_ORDERS`; the native
LS-DYNA node-import branch maps original NID order monotonically to internal
indices. This first profile requires unique ordered four-NID tuples, making the
fifth key irrelevant. It rejects duplicates with original EIDs, without an EID
fallback. Repeated T3 N3=N4 remains in the tuple and later four-slot topology.

Root's source census found all 171,813 original master tuples unique, including
160,896 Q4 and 10,917 T3. Its expected EID permutation digest is
`8e57ad5fc8eee6c58a8bf8518c52813db0e0ce1ca6ecf444ba65ec650f3d5cc6`.
The source declaration retains all 11,165 slaves, including the 908 solid
elements' corners. Packing does not supply their material, volume, mass or
force producer and does not establish a complete vehicle load path.

The startup forecast precedes decoding and optional allocation. It charges
shared canonical/declaration payload once, decoded-record/native-endian
temporaries, and both permutations. Actual permutations require 1,374,504 B;
`owned_payload_bytes` counts their capacities and `PackingData`, excluding
shared backing, allocator/control blocks and RSS. Default caps are 512 MiB,
524,288 masters and 1,048,576 canonical nodes. Factories stage privately and
throw before returning a new handle on any failure; prior handles remain valid.

The native oracle is test-only and separate from production. Its exact source
and context adaptations are described in [native/README.md](native/README.md).
Author host and C++ syntax checks do not establish the Fortran or full-source
gate. Enable that owning gate explicitly:

```sh
cmake -S modelio/tied_shell/packing -B <build> \
  -DCMAKE_BUILD_TYPE=Release -DChrono_DIR=<installed-Chrono>/lib/cmake/Chrono \
  -DROBO_DYNA_TL_ROOT=<TL-root> -DROBO_DYNA_TIED_PACKING_NATIVE=ON \
  -DROBO_DYNA_TIED_CANONICAL=<workspace>/crash-work/assets/yaris-vehicle \
  -DROBO_DYNA_TIED_SCOPE=<workspace>/crash-work/reports/yaris-full-shell-scope-10.json
cmake --build <build> --target robo_dyna_tied_packing_check -j1
ctest --test-dir <build> --output-on-failure
```

The separate source audit in `planning/YARIS_TIED_SHELL_PACKING.md` records
converter/default provenance. `contrl.F` zero-initializes DEF_INTER, and the
audited source contributes no converted `/DEFAULT/INTER` override; explicit
Ignore 2, Spotflag 28 and Idel2 1 survive, while Isearch defaults to 2. The handle
does not manufacture a validated native IKINE/classification packet from that
audit. Complete classification and search associations require separate gates.
