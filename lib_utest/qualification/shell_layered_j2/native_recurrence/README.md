# Independent native layered shell recurrence

This qualification composes retained OpenRadioss geometry, strain, coefficient,
point-law, resultant, stabilization and force leaves for one prescribed shell.
It does not link the full solver, evaluate a production material/force helper,
advance a nodal owner, or claim unrestricted finite-rotation objectivity.

`NativeLayeredReference.h` exposes QEPH and T3 overloads with a borrowed LAW44
curve, existing native reference and prescribed interval, and an independent
three-point history. The result preserves the full native `ForceTrial`, including
world forces/couples, current frame/normals/rates, FOR/FOR_G/MOM/STRA, reported
thickness, signed EINT and QEPH HOURG/EVIS. Separate point and section diagnostics
retain PLA, filtered rates, tangent/yield factors and actual native plastic work.
Every public result is staged; rejected input or native arithmetic leaves output
and prior history unchanged. The caller explicitly accepts the returned history.
These prescribed values are qualification inputs, not authenticated solver
restart records. Curves are borrowed only during the call; no pointer is retained.

The selected scope is centered isotropic IGTYP1, NIP3, ITHK1, LAW44, no damage,
thermal effects, failure, nonlocal plasticity, material rotation/fiber directions,
small-strain overrides or composite sections. The curve stays within its declared
domain, and native thickness clamps are rejected. VP2 uses source C/P and the
native cutoff coefficient and total shell rate; zero C/P/cutoff selects rate off.
Initial zero-rate holds can still yield when a carried VP2 filter lowers yield.
The QEPH branch retains native IDRIL0 projection and full NPT3 stabilization;
this test does not change its domain or authorize removal of any app guard.

## Native composition and provenance

All donors are pinned to `a62b27e6baa555d222a580d6218867d0be4d70b5`, AGPL-3.0-or-later.
`source-manifest.json` authenticates the complete durable MULAWC and LAYINI files
and each unmodified statement excerpt. `prepare_sources.py` verifies hashes and
byte lengths before generating build-only includes, and verifies them again on
every build. The owning QEPH/T3/LAW44 targets independently authenticate their
existing complete leaves; this directory adds no edited duplicate of a leaf.

- `NativeLayeredSection.F90`: actual COQINI position/force/moment tables and
  complete SIGEPS44C via the physical-thickness entry. Unchanged MULAWC lines
  543–547 preserve old work before resultants are cleared (566–582); 682–683,
  767, 850–855 form each physical layer increment; 2010–2018 compute plastic
  work; 2657–2664 accumulate FOR/MOM; 2825–2826/2828 compute mean/min ETSE and
  mean yield; 3016–3032, 3046–3048, 3089–3094 update thickness/DM/work. The driver
  captures material-only FOR before instantaneous DM. Tiny proxy records retain
  the original `lbuf%pla` and `gbuf%wpla` arithmetic without importing full buffers.
- `NativeLayered{Qeph,T3}Section.F`: reuse the existing native PM/context packing,
  then re-execute complete CNCOEF3B/C3COEF3 with MTN44/ITHK1/NPT3, replacing all
  effective coefficients. This extra setup call is qualification packing reuse,
  not a claim of full native dispatch. Complete CZSTRA3/C3STRA3 supplies STRA and
  material-order increments. T3 preserves the source ASRATE1 EPSD overwrite.
- `NativeLayered{Qeph,T3}Force.F`: the existing selected native force composition,
  substituting the layered section call. Complete QEPH CZFINTCE/CZFINTN1/CZPROJN
  receives NEL1/MTN44/NPT3, actual mean/min ETSE and last-point SIGY. Complete T3
  C3SROTO3/C3FINT3/C3FCUM3/C3MCUM3 retains its independent native geometry and
  projection. Original startup nodal mass still feeds QEPH stiffness; it is not
  regenerated from changing thickness. No independent state owner or clock.

Plastic work is the native layer diagnostic in joules. It is compared against
production's density diagnostic times actual force-stage thickness and area;
it is not added to signed material EINT a second time. EVIS is the native signed
stabilization-work channel, not a declaration of purely artificial dissipation.

## Tests and build owner

`LayeredNativeRecurrenceTest.cpp` runs an asymmetric yielded warped Q4 and skewed
T3 through load, hold, time-dependent rigid rotation and unloading, with two
step sizes and separately carried native/production shell and point histories.
The analytic path has endpoint x(t), midpoint v(t) including Omega cross x, and
nodal omega from the same director path. The preloading branch is nonrotating;
rotation starts only after a measurably yielded history exists. No inconsistent
finite-time pure-spin packet or forced zero response is used. Existing point
LAW44 and shell covariance budgets remain unchanged. Forces, couples, geometry,
all twelve QEPH HOURG states, work, thickness and each point/filter are compared.
A separate nonrotating scenario qualifies the rate-off branch with exactly zero
filtered histories. The h/h2 paths are each compared to native; this is not a claimed convergence
order or a comparison to physical measurements.

`LayeredNativeControlTest.cpp` proves wrong midpoint phase and erased yielded
history cause measurable differences. It checks wrong sample/time, a late bad
curve, output/curve alias, and finite third-layer stress that overflows actual
SIGEPS44C after the first two layers execute, with unchanged output/prior history
and exact retry. These controls are prescribed-value tests, not owner rollback.

The owning CMake module is `../LayeredNativeRecurrence.cmake`, included by the
existing shell_layered_j2 owner. `shell_layered_native_reference` is reusable by
source-parent replay qualifications; `shell_layered_native_recurrence_check` is
the CPU-only test target. Disable it explicitly with
`TL_SHELL_LAYERED_NATIVE_RECURRENCE=OFF` when the native reference is unwanted.
When T3 targets already exist, their owner must have enabled T3_R2_BUILD and
T3_R3_BUILD. There is no existing Fortran Bazel owner for this qualification;
this change deliberately adds no foreign-toolchain framework or production link.

Example bounded standalone gate (parent owns heavy/GPU jobs): configure this
parent directory with `TL_SHELL_LAYERED_J2_ENABLE_CUDA=OFF`,
`T3_R2_BUILD=ON`, `T3_R3_BUILD=ON`, the existing local GNU Fortran compiler, and
build `shell_layered_native_recurrence_check` with one process. CTest name matches
the target. The four functions passed in 49 ms; native library build peaked
at 108288 KiB and the two-function test compilation at 284560 KiB, under a
512 MiB address-space limit. The first QEPH attempt caught a new wrapper NEL/MTN
call-slot typo, fixed in this wrapper; failed evidence is preserved separately.
No production mechanics or tolerance was changed to obtain the pass.
