# Original ELFORM9/NIP1 midlayer profile

`ResolveOriginalMidlayer(base, ResolutionProfile::OriginalMidlayerV1)` extends an
authenticated original no-tire V2 resolution. It reads the already retained
PART/SECTION/MAT024 blocks for PID/MID/SECID2000524, checks the exact original
block hashes, units and eight excluded tire IDs, and keeps the immutable base.
No extra JSON artifact or source archive scan is needed. V1/V2 Read/ReadBytes
continue to produce their historical values and topology-based native mappings.

The literal source is ELFORM9, NIP1, .5 mm thickness, blank NLOC, analytic
MAT024 E250/SIGY10/ETAN1 (N/mm²), nu.35, rho1e-9 (t/mm³), FAIL2.5. C/P/VP are
blank; LCSS/LCSR are explicit zero. Raw text, line numbers, blank masks and
named numerical cards stay available. The separate native interpretation is
FilteredZeroC C0/P1/cutoff10000 Hz and ConstantAllPoints D1=2.5; it is not TAB1.
Analytic hardening and virgin A11 use the existing qualified TL preparation.

Both original shapes keep `OneThicknessPoint`, the same original SID/MID and
one thickness station. Native IHBE11 QBAT owns four in-plane stations; the
original triangle EID2357656 uses the qualified T3 one-point formulation. The
resolved QBAT catalog role is supplied by TL's explicit formulation catalog.
This app slice produces source/reference inputs and does not admit that catalog
to a live owner, resident batch, contact or accepted archive.

Three indices remain distinct:

- `parents()[e].topology_index`: unchanged original Q4/T3 position, including
  unavailable rows; the complete source/canonical index and IDs remain intact.
- `native_mapping(e)`: the compiled profile's contiguous available QEPH/T3/QBAT
  position, in canonical source order. Unresolved entries have None/SIZE_MAX.
- `VehicleShellReferences::rows()[e].reference_index`: storage index of a
  successful native startup packet. A rejected packet does not renumber the
  source/native mapping or retain a stale native reference.

`native_parent()` uses the new mapping only for the explicit compiled profile.
The old artifact profiles return null from `native_mapping()` and retain their
historical `native_parent()` behavior. `identity()` still identifies the input
artifact; new interpretation-sensitive association must use `resolution_key()`,
which includes the profile version. No synthetic artifact digest is invented.

The overlay shares old source rows, materials, sections and curves. Its forecast
adds its own control objects, part status copy, complete native descriptors/map,
new declaration and bounded parser/native scratch to the base allowance once.
The existing 512 MiB resolution and 768 MiB explicit reference caps remain.
QBAT reference capacity is counted separately, with exact available-family
counts; unused QEPH slots are not reserved for QBAT parents.

Source conversion preserves the existing binary64 operation order. In
particular, `1e-9*(1000/(.001*.001*.001))` yields 999.9999999999999, not the old
geometry fixture's rounded rho1000 literal. Reference comparison uses the
original fixture's exact positions/IDs and the actual converted material.

Eight new small host tests cover source fields/default separation, late invalid
cards and retry, independent mapping, profile identity, native reference
packing and ordered rejection. Eight prior field/reference tests also run.
Three optional original-source tests are authored for the owning gate: complete
344543 available references, all349645 rows and5102 unresolved rigid rows,
4250 QBAT plus one T3, prior340292 named-reference parity and exact byte caps.
Root qualification now passes all19 functions, including those three original
source tests, in `vehicle-midlayer-root-{source,reference}-tests-1`. Every344543
available reference succeeds, with zero rejections and5102 rigid rows unresolved.
Every prior340292 reference agrees. Source/reference forecasts are340672935 /
599955732 B; the complete reference parity test sampled651325440 B RSS while
holding old and new results together. Source SI operations and admission caps
are unchanged. This establishes source/reference composition, not dynamics.

Build field checks from `modelio/vehicle_sections` with
`ROBO_DYNA_VEHICLE_MIDLAYER_ACTUAL_TESTS=ON` for the two source tests. Build
reference checks from `case/vehicle_startup` with
`ROBO_DYNA_VEHICLE_REFERENCE_MIDLAYER_TESTS=ON` for the full reference test.
Both reuse the existing CANONICAL/SCOPE/DECLARATIONS/RESOLUTION/GLASS_RESOLUTION
paths and explicit GLASS_SHA256 fixture arguments. All outputs remain source
assessment or test evidence, not a completed simulation.
