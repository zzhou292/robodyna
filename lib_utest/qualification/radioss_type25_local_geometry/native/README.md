# Native current-geometry reference

This qualification-only harness extracts pinned OpenRadioss `I25COR3_3` center/normal preparation, the complete `I25DST3_3` raw geometry body, its two initial-offset loops, and the subsequent `I25MAINF` zero-penetration stiffness loop. It never calls the C++ geometry translation.

Current selected barycentric coordinates, source geometry, stored binary32 normals/bisectors, gaps, and incoming stiffness are supplied operands. Search and material-side coefficient initialization are not synthesized. The selected profile is local `IGAP=1`, `ISHARP=1`, `INACTI=5`, `IVIS2=1`, without adhesion or thermal response. All native general, Q4/T3, free-edge and vertex branches remain in the raw source body. The wrapper observes assignment of closest-point scratch and rejects a read before assignment; it does not replace undefined source geometry with zeros.

The history input is **one original logical cohort**, bounded to 256 geometry entries and 256 retained slots for qualification. It performs all TIME0 maxima before any subtraction, followed by the separate native stiffness loop. Repeated history indices preserve original order. A larger production resource ceiling does not authorize merging or splitting native cohorts. Other row fields are retained by copying the input around the original arithmetic.

Existing pinned normal/friction donors are reused by relative path. The additional COR3 source retains its original AGPL notice. Source manifests pin complete donor bytes; generated arithmetic comes only from these files. `prepare.py --output DIR` emits `GeometryReference.F` and `GeometryHistoryReference.F`; `--check` verifies an existing generation. Both require fixed-form Fortran, unlimited fixed-line length, no fast math, and disabled contraction, as in the existing normal/friction qualification. Production consumers do not link either reference function or a Fortran runtime.

Source authored; compilation, numerical parity, and GPU execution remain owning qualification gates.
