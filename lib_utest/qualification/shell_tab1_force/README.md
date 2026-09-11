# TAB1 glass through the existing QEPH/T3 force cores

This increment adds small typed QEPH/T3 history and force wrappers plus one
section adapter. Geometry, material coefficients, local/global force order,
stabilization, native time-step diagnostics and prescribed-history publication
continue through the existing family cores. No family arithmetic is copied.

The supported composition is centered NLay1/NIP3 analytic LAW44 with explicit
FilteredZeroC and TAB1 AnyPoint. The value gates use the original glass
parameters: rho2500kg/m3, E70GPa, nu.22, initial yield30MPa, tangent1GPa,
C0/resolvedP1/cutoff10000Hz and the constant three-point failure table
(-.3,.015),(0,.015),(.3,.015). Geometry is the existing small prescribed Q/T
qualification fixture, not an imported windshield or connected vehicle.
Source NUMINT1 correction and noncentered section placement remain app/source
admission obligations; no resident collection, contact deletion, nodal mass
removal, accepted archive or new mechanics clock is introduced.

TAB1 retains its own uncapped damage, capped DFMAX, failure time, activity and
table cache per point. Saved masked point stress, current unmasked force-point
stress and the final parent mask remain separate. The existing family core
continues rate/geometry/history evaluation after removal; its accepted OFF0/1
and finite geometry contracts remain intact. Removal and later inactive force
values do not imply that source connectivity, contact or mass has been deleted.

## Independent native composition

The test-only source adapter first invokes the unchanged authenticated
shell_failure_force native generator. That generator verifies 13 inputs and
four complete family driver outputs, whose native donor closure remains pinned
to a62b27e6baa555d222a580d6218867d0be4d70b5. The new manifest authenticates the
reuse boundary and all four adapted outputs. Its changes are private symbols,
the explicit four-value table parameter and distinct five-value point failure
history, and a different callback target. Complete native family geometry,
coefficients, force and history operations remain unchanged.

The small TAB1 native callback forwards the actual family M%SSP to the existing
optional native packet seam. It uses the already qualified TAB1 point/parent
callbacks and the same native NIP3 loop. It does not reconstruct a different
sound speed, copy the loop or call production material/force helpers. C++ native
packets retain all native history values independently between intervals.
Native table/point auxiliary fields retain the narrow scope documented by
shell_tab1_glass; these packets are qualification data, not restart files.

## Qualification

Four host functions cover both family startups, all eight selected near-failure
point masks, removal and two later intervals, wrong material policy rejection,
late point failure with unchanged complete output, exact clean retry and
rejection of degenerate geometry after removal.

Two native functions compare complete geometry, shell/current/saved histories,
all point damage/DFMAX/cache values, force/couple arrays, section diagnostics,
work, failure time and parent activity across all eight masks and two later
intervals. Actual returned native M%SSP and viscosity association are checked.
The existing independent force tolerances are reused; no failure-specific
widening is introduced. Two CUDA functions own/update their histories on the
actual device and compare the independent native recurrence, including late
failure preservation and exact retry against an untouched device control.

Author evidence: four host functions pass. Two native C++ translation units,
one CUDA-shaped host syntax unit, native source preparation/check and host
CMake configuration pass. Report:
crash-work/reports/shell-tab1-force-author-1.json (2.720s,246148KiB maximum child
RSS,1CPU/512MiB address-space cap). CUDA-shaped syntax removes launch syntax
only in temporary files; it is not a CUDA execution claim. Fortran/native and
actual CUDA execution are pending the root scheduler.

Root owning commands, under the existing workstation guard:

```sh
cmake -S lib_utest/qualification/shell_tab1_force -B BUILD_DIR \
  -DCMAKE_BUILD_TYPE=Release -DTL_TAB1_FORCE_NATIVE=ON \
  -DTL_TAB1_FORCE_CUDA=ON -DCMAKE_CUDA_ARCHITECTURES=120
cmake --build BUILD_DIR --parallel 2 --target \
  shell_tab1_force_values_check shell_tab1_force_native_check \
  shell_tab1_force_cuda_check
ctest --test-dir BUILD_DIR --output-on-failure -j1 \
  -R '^shell_tab1_force_(values|native|cuda)$'
```

Bazel owning host target:
//lib_utest/qualification/shell_tab1_force:shell_tab1_force_values_check.
The fixture-only Bazel owners expose existing shared test utilities; no native
reference source is linked into production.
