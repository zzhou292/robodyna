# Selected S6Z wedge startup qualification

This is a host/device reference-value slice. It has no nodal ledger, material
admission, element force, contact, collection, or owner publication API.

`solid6z::ReferenceInput` takes six distinct source nodes in paired triangle
order. The selected profile is JHBE24, NIP1, ISMSTR10, IMAS0, FILL1, no thermal,
orthotropic, ALE or alternate-reference options. Virgin global GBUF density is
supplied by the caller; the module does not resolve LAW42 from source cards.
The reference retains source IDs, the native orientation permutation, the
world-reference inverse Jacobian, cyclic local frame/coordinates, native volume,
VZL, characteristic length and six source-slot masses. Scalar inertia is zero.
Every result is staged; rejected input leaves the prior reference unchanged.

The native oracle namespaces complete same-pin `CHECKVOLUME_6N`, `S6ZCOOR3`,
`S6ZRCOOR3`, `S6ZJACIDP`, `S6ZDERI3`, `S6FRACA`, `S6MASS3`, `SLEN`,
`S6CORTHO3`, `SREPISO3` and `SORTHO3` donors. It also retains the referenced
`S6ZORTHO3` module. The packet uses explicit module interfaces, an explicit mass
interface, native array extents and bounds checking. Only context, profile input
and named observation packing are authored. No production helper supplies
oracle geometry or mass. The native IMAS1 control demonstrates unequal angular
weights; production explicitly rejects this unselected policy.

The four tiny host functions qualify value invariants, orientation association,
late mass overflow/retry and the explicit raw-slot topology mapping. Three native
functions compare complete selected values, the distinct IMAS branch and a
first-face Y-order control. One source-native function compares all 195 mapped
original cells, both directly in SI and after an independent working-unit native
evaluation. Three CUDA functions perform device-owned startup, all 195 source
packets, and late rejection/retry against the independent host native oracle.
Native/Fortran, CUDA and original full-count execution are root qualification
obligations; the author ran only four small host functions and C++ syntax checks.

The source fixture preserves all original eight slots and source line/blank-mask
evidence. Its 195 collapsed cells have `[A,B,C,D,E,E,F,F]`. The **explicit demo
topology mapping** is `[A,B,E,D,C,F]`, before native orientation correction;
`MapCollapsedTopEdges` retains all eight original IDs and six index associations.
Five nondegenerate faces and nine edges are preserved. This is not a claim that
the pinned converter itself recognizes or performs that conversion.

The source reader investigation is recorded in `SOURCE_PROFILE.md`. The demo
chooses the S6Z profile explicitly while preserving the original physical
material/load paths. Original IHQ2/QM0.1 is never copied into a HEPH hourglass
coefficient. The eventual case producer must authenticate and name that policy.

Configure this directory with `TL_SOLID6Z_REFERENCE_NATIVE=ON` and
`TL_SOLID6Z_REFERENCE_CUDA=ON` for owning gates. Test-only native donor sources
never enter the production Bazel `//lib_src/elements/solid6z:reference` target.

Root `rubber-wedge-root-tests-1` passes11 numerical functions (4 host,4 native,
3 CUDA) plus2 identities at integrated `cdbe23b`. All195 original mapped
wedges require the same native orientation reversal; wedge-only mass is
0.056384 kg. This qualifies startup, not force or complete source admission.

Affected legacy HEPH10 numerical functions and2 identities PASS in
`wedge-affected-solid24-root-tests-1`; owning S6Z/affected-reference builds PASS
in `cin-wedge-owning-bazel-build-1`.
