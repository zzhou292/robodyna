# Selected original T3 one-point LAW44 recurrence

This pure prescribed-input gate starts from TL ffbc12d. It implements the single
original EID2357656 in PID2000524 as its own NPG1/NPTT1 native formulation:
ISH3N2, ISMSTR2, ITHK1, centered, GS0, DM0 and IDRIL0. It does not change the
existing integrated LAW1 or layered NIP3 entry points. There is no resident
batch, source population admission, owner, contact participation or new clock.

`T3OnePointTypes/History` owns one genuine point history plus native element
FOR/FOR_G/MOM/GSTR/THK/EINT/EPSD/OFF, D1 damage/time and WPLA. Immutable reference,
material/curve backing and failure parameters bind history scope. The caller
owns endpoint x and midpoint v/omega and supplies consecutive interval stamps.
Prepared material curve arrays must remain immutable and alive. SI units and
native positive-internal-force convention match the existing T3 interface.

`T3OnePointCoefficients/Material/Force` preserves this sequence:

1. Current native frame, old reported force thickness and NPT1 coefficients.
2. All eight native C3DEFO3/C3CURV3/C3STRA3 increments and curvature-dependent
   C3FORC3 rate, even though centered NPT1 has zero bending resultants.
3. One LAW44 point at z=0, native elastic then plastic physical-thickness
   updates exactly once, rounded accumulated-PLA work and constant D1 failure.
4. FAIL_SETOFF_C parent transition and final cached-force mask. The removal
   interval retains unmasked current point stress and old cached-force work;
   the final world force/couple is zero. Following intervals retain zero cache.
5. Actual post-SIGEPS44C sound speed, C3DT3 positive active STI/STIR and native
   DTEL, then complete native stress rotation/local force/world projection.

SHF/GS0 does not erase transverse increments or curvature history. The named
shared `UpdateLaw44ZeroShearPlasticity` accepts the supplied finite five-strain
packet at GS0; `UpdateLaw44MembranePlasticity` retains QBAT's zero-transverse
packet guard and the legacy shell entry still requires positive GS. Actual
virgin one-point histories retain zero transverse stress because GS is zero,
not because values are reset. Imported histories must obey that named domain.

One-point WPLA and EINT are separate native bookkeeping channels. DTEL/STI/STIR
are observations, not new owner stability admission or an energy tolerance.
All public outputs publish once after complete validation; aliases, invalid
scope/phase and late nonfinite arithmetic preserve the prior output.

## Independent native authority

All donors use OpenRadioss a62b27e6baa555d222a580d6218867d0be4d70b5. The native
manifest hashes complete retained files and exact source-line fragments. The
existing native T3 owner supplies complete C3COOR3/C3EVEC3/C3DERI3/C3DEFO3/
C3CURV3/C3COEF3/C3STRA3/C3DT3/C3SROTO3/C3FINT3/C3FCUM3/C3MCUM3 leaves.
Private readable call adapters set the selected actual flags, including OFF0
follow-ups, without changing the old native adapters. C3FINTRZ is excluded by
the actual IDRIL0 branch. Native `ISMDISP=0` is explicit.

The shared `qualification/native/law44_one_point` owner contains the complete
LAW44 and failure leaf calls plus exact MULAWC saved-stress, thickness,
resultant and work excerpts. Its explicit `npg` argument retains the native
`NPG>1` old-work mask only for QBAT. T3 uses 1. All point, parent and work
histories advance independently in the native oracle after initialization.
The new caller carries actual `values(11)` SSP from SIGEPS44C into C3DT3.
Legacy Q/T post-law SSP corrections are a separately owned audit/change.

The fixture reuses `qualification/qbat/source_fixture`, its sole original
owner and narrow test-only Bazel target. The authenticated header is 659264 bytes,
SHA256 `87c3902373d3a5e9e27c09522ad9adc5f4c6e6822eb2f64aa613d82bc1638c98`.
Its manifest retains the one excluded-from-QBAT triangle, canonical parent
228325/source line 274492, original material/section cards and full source
hashes. The selected nodes are 2300357/2300138/2300139, with node 3 repeated in
the original four-slot shell card. Tests use actual 0.5 mm thickness, rho 1000,
E 250 MPa, nu .35, SIGY 10 MPa, ETAN 1 MPa, filtered C0/P1/cutoff 10000 Hz and D1=2.5.
Prescribed rotations/yield paths are qualification inputs, not a vehicle run.

## Checks and root gate

Author checks: four new host functions and all four affected QBAT host
functions pass under one CPU/512 MiB. Native C++ oracle/tests pass bounded
syntax; native donor/fragment and legacy T3 identities pass. Reports:
`/tmp/t3-one-point-host-tests-3.{json,xml}`,
`/tmp/t3-one-point-qbat-tests-1.{json,xml}`,
`/tmp/t3-one-point-native-syntax-1.json`.
The first removal test used a uniaxial stress that unloaded on this path;
its failed report 1 remains. The corrected test seeds a valid biaxial near-D1
state. No production arithmetic or comparison tolerance changed for that fix.

Five native tests cover independently carried 256-step original/transformed
geometry yield/unload/reload, all eight nonzero strains and changed thickness,
removal/current-work/next-zero packets, table/positive-rate/rate-off branches,
wrong phase/reset-history/exact native retry controls, and direct native GS0
with nonzero transverse increments plus carried transverse stress. Two CUDA tests
cover independent device history and late material failure/phase rollback
with preserved buffers and exact retry. These native/GPU tests are authored
but not executed by this author.

Root-scheduled owning commands (use existing bounded runner/native compiler):

```sh
cmake -S lib_utest/qualification/t3_one_point -B BUILD_DIR \
  -DCMAKE_Fortran_COMPILER=PINNED_GFORTRAN \
  -DT3_ONE_POINT_NATIVE_CHECKS=ON -DT3_ONE_POINT_CUDA_CHECKS=ON \
  -DCMAKE_CUDA_ARCHITECTURES=120
cmake --build BUILD_DIR --parallel APPROVED_WORKERS
ctest --test-dir BUILD_DIR --output-on-failure
```

This also owns affected QBAT native/host, material/failure and existing T3
startup/geometry/global-force regression groups. Host-only config sets both
one-point checks OFF. Bazel production is `//lib_src/elements/t3:one_point_law44`;
tests are `//lib_utest/qualification/t3_one_point:t3_one_point_check` and
`//lib_utest/qualification/t3_one_point:t3_one_point_cuda_check`.

Root integration passes11 owning functions:4 host,5 native and2 actual CUDA,
plus10 affected QBAT host/native functions and2 source identities. Reports:
`t3-one-point-root-tests-2` / functions2, build3. Native build1 found unprefixed
ELBUFDEF/ELEMENT module imports in the new wrapper; those now name the already
private T3 engine modules. No donor routine or production equation changed.

Tests1 had complete native arithmetic agreement, but one rotated load path
produced only0.947 micrometres of thickness excursion, below its asserted
1 micrometre coverage threshold. That one prescribed path now uses amplitude
0.30 instead of0.18; default paths, the threshold and all comparison tolerances
remain unchanged. The stronger path passes the same full-history comparison.
Native loops now stop after the first mismatch to bound failure logs.

The source fixture is shared with its sole QBAT owner. This gate does not
independently qualify the original triangle's startup M/J or a resident owner;
the explicit native startup comparison belongs to the next resident gate.
