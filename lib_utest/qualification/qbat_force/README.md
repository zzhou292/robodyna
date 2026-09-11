# Selected QBAT four-point native force gate

Base: TL 22f0755, including the frozen QBAT geometry and its corrected native
loop closure. This gate adds a pure four-surface-point recurrence; no owner,
resident batch, full-vehicle source admission or physical energy tolerance.

Production uses small geometry/velocity/strain/material/viscosity/force
modules. The shared GS0 material entry leaves the old shell positive-GS
contract unchanged. `sections/ShellLaw44MembranePoint.h` contains the reusable
one-thickness material/thickness/failure packet; QBAT owns sequential quarter
thickness, constant-shear work, IPG4 parent removal and cached-force stages.

## Independent reference

All native donors use OpenRadioss
`a62b27e6baa555d222a580d6218867d0be4d70b5`.
`native/source-manifest.json` authenticates complete donors by Git blob, size
and SHA256 and every exact selected fragment by source lines and bytes.
`native/LICENSE.md` retains AGPL3; original source headers retain attribution.

- `NativeKinematics.F`: exact CBACOOR current frame, corrected velocities,
  actual spin, point geometry and current length. Existing complete CBADEF1/
  CBADEFSH and engine CLSKEW3 remain under the qualified geometry owner.
- `NativeMembranePoint.F90`: complete SIGEPS44C and FAIL_JOHNSON_C through
  existing native owners, plus exact MULAWC physical-thickness/work/saved-
  stress/resultant operations. Its point and force-work routines are separable
  from QBAT's four-point coordinator.
- `NativeParent.F`: exact selected FAIL_SETOFF_NPG_C operations over a minimal
  shaped storage view. The source PTHKF=1/NPTT1/one constant criterion is
  explicit. Only logging and disabled failure-wave statements 197:202 are
  omitted; source branch closures 203:206 remain exact.
- `NativeForce.F`: selected original stage order, complete CBASTRA3,
  CBAENER(S), CBAFORI1/CT, CBAVISC/CBAVISNP1 and CBAPROJ routines. Exact CNCOEF3 material/GS/DN, rate,
  quarter-thickness and CNDT3 excerpts retain native expression order.

No production frame, derivative, rate, failure decision, force or work value
is supplied as expected output. Native initial histories are copied once;
all subsequent native histories advance independently. The C++ packing is a
test-only named-field representation (116 state scalars / 266 observations).
Native point mass, ownership and physical energy balance are outside scope.

Selected original defaults: E=250 MPa, nu=.35, rho=1000 kg/m3, t=.5 mm,
SIGY=10 MPa, ETAN=1 MPa, explicit filtered C0/P1/cutoff10000 Hz and D1=2.5.
The source authority is the existing authenticated 4250-quad midlayer fixture;
the single original T3 remains outside this QBAT gate. The source sample
test checks first/final and largest warp/smallest startup DT geometry. This
is not a source trajectory. Finite near-failure packets deliberately exercise
the native point-order transition without a whole-vehicle admissibility claim.

## Coverage and author checks

Four host functions pass, including a 256-interval yielded rotate/unload/
hold/reload path, named GS0 versus unchanged legacy entry, each last-failing
point, late fourth-point stress and cached-energy overflow, phase/scope errors,
aliased publication and exact retry. Owning host CMake/CTest:
`/tmp/qbat-force-host-owning`, report
`/tmp/qbat-force-host-owning-2.xml`; one CPU and 512 MiB virtual-memory limit.
Native C++ oracle and source-sample units pass bounded syntax. Source hashes
and exact included DO/IF nesting were checked; this is not Fortran execution.

Six native functions are authored: independently carried trajectories over
flat/warped frames and nonzero DM, all16 initial failure masks with two OFF0
follow-ups, table/rate history, wrong duration/frame/reset-history controls,
complete GS0 carried transverse stress, and selected original geometries.
Two actual CUDA functions cover independent histories/removal and failed
fourth-point material/work stages, changed sample, preservation and exact retry.
Native, CUDA, and Bazel execution are root-owned and remain pending.

## Root qualification commands

Use the existing pinned native compiler environment and bounded workstation
runner. Explicit Fortran compiler is required in this workstation setup.

```sh
cmake -S lib_utest/qualification/qbat_force -B BUILD_DIR \
  -DCMAKE_Fortran_COMPILER=PINNED_GFORTRAN \
  -DQBAT_FORCE_NATIVE_CHECKS=ON -DQBAT_FORCE_CUDA_CHECKS=ON \
  -DCMAKE_CUDA_ARCHITECTURES=120
cmake --build BUILD_DIR --parallel APPROVED_WORKERS
ctest --test-dir BUILD_DIR --output-on-failure
```

The owning gate also runs the prior native geometry/original geometry,
constant failure, native LAW44 point/rate/analytic and shared quadrilateral
tests. Host-only configuration uses both new checks OFF.

Bazel targets:
`//lib_utest/qualification/qbat_force:qbat_force_check`,
`//lib_utest/qualification/qbat_force:qbat_force_cuda_check`;
production `//lib_src/elements/qbat:force`.
No force or history value is admitted to a live owner by this gate.

## Root integration qualification (2026-09-10)

The combined TL gate passes **70 numeric functions**:12 new (4 host,6 native,
2 actual CUDA) and58 affected geometry, QEPH, LAW44 and constant-failure functions,
plus3 source identity checks. Reports are `qbat-force-root-{configure,build,tests}-*`;
the accepted result is tests3 / functions3. No skipped functions or tolerance changes.

The first native execution exposed an uninitialized common NPSAV used for local
EVIS allocation. The test wrapper now uses eight fixed rows and initializes the
native stride before calling leaves. The second execution caught a production
omission: complete CBAPROJ multiplies final world forces/couples by final parent
OFF. That final mask is now applied after projection; earlier Gauss stress
caches and work remain as computed. Tests1 and2 are retained failed evidence.

The shared reference now also carries explicit placement. QBAT keeps its
qualified centered-only startup scope; force-history identity compares placement.
No original source collection, resident owner or trajectory is admitted here.
