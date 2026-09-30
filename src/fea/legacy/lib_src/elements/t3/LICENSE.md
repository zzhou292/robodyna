# T3 port provenance and license

These headers adapt the selected OpenRadioss T3 routines and source-expression
adapters at commit `a62b27e6baa555d222a580d6218867d0be4d70b5`, copyright
(C) 2026 Siemens, under **AGPL-3.0-or-later**. The complete license is retained
at [the pinned original license](../../../lib_utest/qualification/native/qeph/original/LICENSE.md).
The source and transformation inventory is
[`qualification/t3/source-manifest.json`](../../../lib_utest/qualification/t3/source-manifest.json).
The native reference remains independent and unchanged.

Scalarization removes MVSIZ loops, inactive branches, untouched material
directions, COMMON state and unused scratch. The selected ordinary binary64
arithmetic, source component order and constants remain. The qualified branch
uses three native nodes, IGTYP1, IREP0, IFRAM_OLD1, ISH3N2, ISMSTR-1 and IRESP2.
It does not implement alternate formulations, source MAT024/NIP3, or dynamics.

Shared `math/Fixed3.h` records and `math/Quaternion.h` finite/rotation utilities
retain their own Project Chrono BSD notices. They are not replaced or relicensed.
