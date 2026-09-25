# TYPE25 startup coefficient arithmetic

`RadiossType25Coefficients.h` exposes small host/device leaves for an explicitly supported native branch. They allocate nothing and add no state owner, clock, search, history or incident-roster reduction. Production links only the existing TL math/normal interface, never the native Fortran reference. Siemens OpenRadioss a62b27e6 source attribution and AGPL license are retained in the parent directory and owning qualification donors.

`MainShell.h` preserves the ordinary PM20 material branch and element/property thickness precedence. `MainSolid.h` consumes authentic face area and solid volume, selects PM107 only for ICONTR1, and keeps the native higher-order corrections. Its length output is a solid characteristic length, not a fabricated contact clearance. Coating, internal-face averaging/signs, stack branches and negative prescribed scale are unsupported.

`Nodal.h` finalizes already accumulated global physical-model quantities with ASSTIFI, including previous STIFINT contributions. Bulk-times-volume input and normalized pressure output are distinct fields. Complete model incidence and global once-per-node ICONTR corrections remain the source owner's responsibility. The separate secondary projection preserves zero removal masks and positive/default source scaling.

`Pair.h` requires an explicitly resolved IGSTI4 / ISTIF_MSDT0 profile and returns the source min/abs/clamp coefficient. It does not apply history, halve stiffness, add damping, or assert that a contact is active. Those are separate qualified response stages.

Every output is staged and published only on success. Native/Si types prevent accidental domain mixing. SI conversion uses existing unit utilities and dimension-specific area, volume and pressure factors; native denominator floors are applied only after conversion. This bounded numerical admission rejects nonfinite arithmetic and unsupported branches without claiming identical unrestricted-Fortran error behavior.

See `lib_utest/qualification/radioss_type25_coefficients/README.md` for the independent source oracle, case coverage, T3 source-selection caveat, tests and remaining physical binding gates. Passing this value layer does not establish complete vehicle startup coefficients or performance.


`SolidNodal.h` and `SpringNodal.h` provide the separate native startup contact
contribution values. Solid shares retain the H8/Penta6 raw occurrence mask;
spring STR consumes resolved channels and the source length-mode policy. They
do not perform whole-model accumulation, infer SI spring slope units, or replace
ASSTIFI/beam STP/structural STI. All invalid/nonfinite results remain unpublished.
