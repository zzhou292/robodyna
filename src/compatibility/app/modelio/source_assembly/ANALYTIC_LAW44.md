# Original analytic LAW44 assembly input

The existing assembly compiler and C++ reader now have an explicit V2 material
mode. Default compilation remains V1/table-only. Existing frozen V1 files and
their declared hashes remain unchanged; regenerated reports record current
generator hashes as usual.

Select `--material-policy law44_tabulated_or_linear` on
`tools/compile_yaris_assembly.py`. V2 requires `hardening_model` on every material:
`law44_tabulated` retains a real curve, while `law44_linear` requires source
LCSS0, supplied SIGY/ETAN and an explicit null parent `curve_index`. Source curve
ID zero means no curve. C++ uses `NoCurveIndex` only after checking that
combination and never indexes a curve vector through it.

`ReadLaw44Material.cpp` shares source-card/unit checks. Positive C/P, VP0,
blank failure/deletion/LCSR/inline fields and NIP3 restrictions remain. The
converter policy fixes A=SIGY, B=ETAN*E/(E-ETAN), n=1 and the 10000 Hz default
filter in source seconds. `SourceAssemblyMaterialInput` forwards original
scalars and the tag to TL's qualified catalog. Geometry, IDs, raw cards and
interface inventories use the same blocks for both versions.

Actual fixtures under `crash-work/reports`:

- `yaris-analytic-assembly-inventory-1.json`: complete PID2000064,
  358 QEPH / 30 T3 / 421 nodes, no curve, two outgoing rigid groups and
  33 outgoing spotwelds.
- `yaris-analytic-mixed-assembly-inventory-1.json`: that part plus original
  tabulated PID2000157, 446 QEPH / 36 T3 / 538 nodes, one real curve.

The owning `source_analytic_input` check authenticates both pinned byte/hash
identities, prepares every native reference/material mapping, checks null/table
ownership and rejects changed modes, scalars, last-parent curve associations and
filter policy. Existing six/seven-part input tests remain enabled. Python tests
preserve V1 geometry/attachments and reject failure or missing-curve substitutions.

This qualifies source input and native host preparation. It does not establish
closed connections, a new accepted trajectory, full-shell resident capacity or
V2 accepted-archive replay. The full 52,483-shell analytic population remains a
larger runtime gate; these actual fixtures exercise its reusable assembly blocks.

Configure `modelio/source_assembly` with the existing TL/Chrono and six/seven
paths plus `ROBO_DYNA_SOURCE_ANALYTIC_INVENTORY` and
`ROBO_DYNA_SOURCE_ANALYTIC_MIXED_INVENTORY`; build
`robo_dyna_source_analytic_input_check`. This host gate needs no CUDA or Fortran.
