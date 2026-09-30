# Selected QBAT source contract

All native donors use OpenRadioss revision
`a62b27e6baa555d222a580d6218867d0be4d70b5`. Complete files and exact byte ranges
are retained in `native/source-manifest.json`; the native reference links the
unchanged, independently verified `qualification/native/qeph` owner for
CNEVECI/CDERII/CINMAS and engine CLSKEW3. There is no production-reference
self-comparison. Native wrappers only initialize context, select audited
branches, call native leaves, and pack outputs.

## Source and defaults

The original PID/SECID/MID2000524 has 4,250 quads and one triangle. Its retained
fixture manifest contains complete original PART, SECTION and MAT024 blocks,
source ZIP/key/README hashes and canonical array identities. Quads and their
4,384 nodes retain source order, canonical index, source line, original IDs and
exact SI coordinate bits. The omitted triangle EID2357656 is explicitly listed.
The original section thickness is **0.5 mm**; rho/E/nu are **1000 kg/m³,
250 MPa, .35**. No geometry-only result admits its material/failure/runtime role.

`convertprops.cxx:774–820` maps the source ELFORM9/NIP1 to external Ishell12,
Ish3n2, Ismstr2, ITHICK1/IPLAS1. `hm_read_prop01.F:306–315` translates external
Ishell12 to internal IHBE11. The converter's explicit NIP value is retained;
the later `else if (elform==9)` is not reached because category1 already matched.

The retained `defaults_mod.F90:122–175` sets Idrill0 and changes it to1 only for
implicit mode. `hm_read_prop01.F:214–216` preserves resolved zero in the selected
explicit path. `prop_p1_shell.cfg:91–99` supplies Hm/Hf/Hr=.01 and no Dm/Dn value.
For external Ishell12, `hm_read_prop01.F:228–230` overwrites GEO13 with GEO17/Dn.
Its `DN_P=EM03` at280 only changes a printed local value, not GEO13.
`set_elgroup_param.F:80–123` leaves LAW44/IHBE11 default membrane viscosity zero.
The API therefore carries the **resolved** CNDLENI inputs explicitly, with zero
for this source branch; positive explicit resolved values are also value-tested.
It does not inherit QEPH's .015 coefficients or infer later force viscosity.

## Reference and current arithmetic

* `CBAINIT3:268–320` orders CCOORI, CNEVECI, CINMAS, CDERII and CNDLENI. Existing
  common startup arithmetic is applicable: CINMAS853 uses rho*t*A/4;
  925–932 selects FAC12 for IHBE>=11; 1381–1390 gives the centered native total
  scalar inertia. No offset, mass scaling or alternate INER_9_12 is admitted.
* `CNDLENI:307–393,440–455` supplies the selected LAW44 sound speed, IHBE11
  FAC11=4/3, viscosity factor and nodal stiffness. Its local `VISCE=EM3` at361
  is subsequently overwritten by GEO13 at383. PM24 is a resolved input, not
  synthesized from a substitute material. Native stiffness/dt use the same
  exact reference area/local-coordinate packet produced by native startup.
* `CBACOOR:201–265` uses current sum/subtraction R/S order, engine CLSKEW3 K0,
  current mean plane, area and raw ZL1. This is the same current-frame leaf
  already used in QEPH, whose remaining geometry/force code is not invoked.
  OFF1/ISMSTR2 uses current geometry; OFF2 and saved-small-strain substitution
  remain outside this increment. Isotropic IREP0 adds no CORTDIR3 axis change.
* `CBACOOR:349–371,440–477,513–548` gives the centered projection, native
  NPT1/NPINCH0 branch selection, packed VCORE and four JAC/HX/HY values.
  Geometrically warped source quads still enter the flat algorithm because
  NPT==1. No ZL1 value is silently erased.
* Complete `CBADEF1:1198–1263` supplies the four BM(1:8) rows. Complete
  `CBADEFSH:723–768` independently exposes constant shear using four velocity
  basis probes in the oracle. Production returns coefficients only, not a
  physical velocity/rate observation.

The source's unsuffixed PG `.577350269189626` and CNDLENI coefficients `3.413`,
`.7071`, `.78`, `.22` are default REAL32 values promoted to MYREAL8. Production
preserves those values; the Fortran oracle retains the literals unchanged.
Named FOUR_OVER_3, ONE_OVER_12 and EM20 use the retained native constant module.

## Deferred caller dependencies

CBACOOR373–437 projects actual staggered velocities and applies DT1-dependent
first-order frame correction, including guarded denominator branches. These
operations must be added with their native phases before any deformation or
force driver. CBAFORC3 then needs four independent material points, sequential
quarter-thickness updates, the constant-shear energy split, CBAVISC/CBAVISNP1,
CBAFORI1/CBAFORCT, CBAPROJ, work and failure masking. No local geometry proof
can replace that independent native caller qualification or once-only nodal
M/J/publication integration. The source's failure and rate modes are not
changed by this startup leaf.

## Qualification

`qbat_geometry_host_test`: six independent ledger/domain/rounding/failure tests.
`qbat_geometry_native_test`: four native cases, including 96 transformed and
deformed packets, source viscosity inputs, large world translations, distinct
starter/current normalization, and rejected candidate retry.
`qbat_original_geometry_test`: all 4,250 original reference/current packets
plus first/last/most-warped parent motion and retry controls.
`qbat_geometry_cuda_test`: two optional actual device cases, 96 packets and
late reference failure/current rejection/exact retry. No device skip is used.
The native harness also runs both existing native QEPH test groups.

Author evidence: six host functions pass in `/tmp/qbat-host-check-2.xml`;
native C++ test sources pass syntax; both source checks pass. A bounded host
preflight admits all 4,250 original geometry packets with quad mass
0.53508621210713381 kg, minimum initial element dt 1.5429147565132538e-5 s and
maximum raw warp 2.1697457886104242e-4 m. This is not native/CUDA evidence.
Native/actual CUDA runs are coordinator-owned after freeze.

Root qualification now passes all 14 new numeric functions: six host, four
native, two complete original-source and two actual CUDA. All 23 existing native
QEPH kinematics/force functions also pass, with zero failures or skips in
`crash-work/reports/qbat-geometry-root-functions-1`. Both source identity gates
pass. Reports are `qbat-geometry-root-{configure,build,tests}-*.json`.
The first native build exposed an extraction ending one line before its closing
ENDDO; `22f0755` restores that exact source boundary through CBACOOR265. The
second build and first complete CTest run pass without production changes or
tolerance changes. Geometry admission still does not qualify the force caller.

After QEPH acquired an explicit reference-plane placement field, QBAT's wrapper
also explicitly rejects noncentered quadrilateral inputs. Its own selected
offset ratio remains zero; broadening the shared startup helper must not expand
this formulation's scope. One host and one actual CUDA rejection/retry function
cover both placement signs. All16 QBAT functions and23 native QEPH regressions
pass with the new identity layout in `qbat-placement-scope-root-functions-1`.

```sh
cmake -S lib_utest/qualification/qbat -B BUILD \
  -DQBAT_NATIVE_CHECKS=ON -DQBAT_CUDA_CHECKS=ON \
  -DCMAKE_Fortran_COMPILER=EXISTING_GFORTRAN -DCMAKE_CUDA_ARCHITECTURES=120
cmake --build BUILD --parallel 1
ctest --test-dir BUILD --output-on-failure
```

Bazel host/device owners are
`//lib_utest/qualification/qbat:qbat_geometry_check` and
`//lib_utest/qualification/qbat:qbat_geometry_cuda_check`. No native download,
full solver build or installed compiler change is part of this harness.
