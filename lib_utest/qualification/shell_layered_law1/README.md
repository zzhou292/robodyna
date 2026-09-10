# Native layered LAW1 point and NIP3 qualification

This value-only increment implements the native elastic `SIGEPS01C` point and
centered NIP3 `MULAWC` section for `ISMSTR=2`, `OFF=1`, and evolving physical
thickness (`ITHICK=1`). It does not admit an elastic catalog entry or change a
resident batch. QEPH/T3 force, stabilization, work and rotating-history adapters
are the next separate gate. No whole-shell or full-vehicle readiness is implied.

`ShellElasticLaw1PointHistory` owns exactly the five elastic stresses. There is
no plastic strain, yield, tangent-hardening, or rate-filter history. This is a
distinct material branch, not LAW44 with an artificially large yield limit.
`PrepareShellElasticLaw1Point` reuses the already qualified native LAW1
coefficient producer. The old global `SIGEPS01G` helper remains unchanged.

`ShellNip3.h` owns the existing centered three-point positions and force/moment
weights, layer increments, and source-ordered resultants. The old J2 public
functions delegate to those helpers without changing any expressions or
reduction order. Native `WM` retains the rounded default-real `0.0833333f`;
substituting exact `1/12` or `WF*Z` would change the formulation.

The section's `reference_thickness` is the effective force thickness supplied
for this interval, normally the previous accepted physical thickness. It stays
fixed while all three layer strain increments and layer volumes are formed.
The separately supplied `reported_thickness` is updated in point order by
`THK += EZZ * THKLY * OFF`. The outputs called `material_stress` and
`bending_stress` are native stress-like `FOR` and `MOM`; physical membrane/shear
resultants are `FOR*t`, and bending/twisting resultants are `MOM*t*t`.

Inputs require finite positive coefficients, density, shear modulus and
thickness, `0 <= nu < 0.5`, and finite increments/stresses. Both value and native
wrapper scopes reject any nonfinite result or running physical thickness below
`1e-30 m`. This is explicit admission validation; the original native routine
itself is retained unchanged and does not supply that rejection. Section output
publication occurs only after the last point/resultant succeeds. All borrowed
inputs are consumed before publication, so an aliased output does not alter an
input still needed by the computation.

## Independent native ownership

The durable donor and exact-byte fragment manifest are in
`../native/law1`. The new original `sigeps01c.F` is from OpenRadioss commit
`a62b27e6baa555d222a580d6218867d0be4d70b5`, 6250 bytes, Git blob
`e0e585dbe7c509fa5c6eb7d15155226e09ad31c0`, SHA256
`66768b206bfd5110b9babc24679c1d0614fd39ecb30b3241442998f6a41353b7`.
`prepare_sources.py` authenticates the complete original before renaming only
its leaf symbol and private `PARAM` common. The shared native LAW44 target owns
and authenticates the existing constants/includes and exact `COQINI` tables;
no LAW44 material evaluation supplies the elastic point oracle.

`NativePoint.F` uses the exact `HM_READ_MAT01:117–134` coefficient statements
and invokes the complete original `SIGEPS01C` routine. `NativeSection.F90`
uses independent point histories, actual `COQINI`, and exact `MULAWC` excerpts
for layer thickness (767), layer increments (850–855), and resultants
(2657–2664). Each running thickness is returned for independent wrapper scope
checks. The C++ oracle wrapper calls no production material/section routines.
Its native common-block context is test-only and serial.

The host tests check an independent long-double scalar oracle, native bending
weights, exact coefficient admission, physical thickness, late third-layer
overflow, unchanged outputs and retry. The native tests compare independent
loading/holding/reversed histories, coefficients, all point stresses,
resultants and thickness; resetting histories or freezing the force thickness
must produce a detectable difference. The CUDA test repeats the history path
on the actual device and checks late failure preservation. This does not yet
qualify time-dependent rotating QEPH/T3 geometry or shell energy accounting.

## Owning gates

Host-only author check (1 CPU, 512 MiB):

```sh
cmake -S lib_utest/qualification/shell_layered_law1 \
  -B /tmp/tl-layered-law1-host-1 -DTL_LAYERED_LAW1_NATIVE=OFF
cmake --build /tmp/tl-layered-law1-host-1 --parallel 1
ctest --test-dir /tmp/tl-layered-law1-host-1 --output-on-failure
```

Root-scheduled complete native/CUDA gate:

```sh
cmake -S lib_utest/qualification/shell_layered_law1 \
  -B BUILD_DIR -DTL_LAYERED_LAW1_NATIVE=ON -DTL_LAYERED_LAW1_CUDA=ON \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_CUDA_ARCHITECTURES=120
cmake --build BUILD_DIR --parallel 1
ctest --test-dir BUILD_DIR --output-on-failure
```

The native configuration also owns the existing LAW44 point/rate/analytic
checks and J2 section/covariance regression tests. Bazel owns the light value
gate `//lib_utest/qualification/shell_layered_law1:shell_layered_law1_value_check`.
The material BUILD registration has a separate T3 provenance amendment; the
original force/native donor records remain unchanged.
