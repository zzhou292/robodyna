# Native layered LAW1 QEPH/T3 force adapters

These are value-only adapters for the selected centered, isotropic elastic
`LAW1 / NIP3 / ITHICK1 / ISMSTR2 / OFF1` branch. Each adapter composes the
existing family geometry, material coefficients, force projection, stiffness
and work routines with the independently qualified `SIGEPS01C` elastic point
and three-point section. There is no resident-batch/catalog admission, second
clock or mass producer. Full-source elastic parts still require a separately
qualified heterogeneous catalog/batch dispatch and case/output declaration.

`LayeredLaw1History` retains the native family history and a distinct sidecar
of five stresses per point. It has no PLA, yield, plastic work, material rate
filter, or plastic stabilization diagnostics. Matching section resultants and
original E/nu/rho are required before evaluating an interval. The force-stage
thickness is the prior accepted physical thickness; the section's updated
physical thickness is proposed with the same family history/sample. A late
failure publishes neither history nor forces.

`ShellLayeredWork.h` contains only law-independent material identity,
resultants identity and the existing native generalized stress-work arithmetic.
The old J2 API delegates to it with identical expressions and ordering. The
elastic adapters use the same work update, including signed instantaneous
membrane viscosity work. QEPH retains the existing elastic stabilization and
its signed total stabilization/viscous work channel. These channels are not all
plastic or dissipated energy. T3 retains its native total equivalent-rate
family diagnostic (`C3EPSD=ONE`) without creating elastic point rate history.

## Independent native force-stage oracle

The existing QEPH/T3 native owners provide complete pinned geometry,
coefficient, stiffness, force, projection and history leaves. Their existing
LAW44 driver and tests remain untouched. The new small dispatch wrappers
select `MTN1`, `ITHK1`, `NPT3`; they call the new independent native LAW1
section and exact `MULAWC` work/thickness excerpts, with their own accepted
five-stress point histories. No production material/section/force evaluator is
used to produce reference results.

All donors use OpenRadioss revision
`a62b27e6baa555d222a580d6218867d0be4d70b5`. `native/source-manifest.json` retains
an authenticated complete cached `CZFORC3` copy and its original Git blob for
the elastic stabilization default (`SIGY=EP30`, lines 384–390). This is the
native dispatch sentinel that bypasses plastic stabilization; it is not an
invented material yield state and is not exposed by the elastic adapter.
`MULAWC` leaves that sentinel unchanged for the selected LAW1 QEPH branch and
accumulates unit ETSE factors with the native weights. Setting this native
sentinel to zero would incorrectly activate the plastic correction.

The point owner authenticates `SIGEPS01C` and `HM_READ_MAT01`. The existing
layered recurrence owner authenticates `COQINI`/`MULAWC` and prepares the exact
`WorkBefore`, `ThicknessViscosity`, `WorkAfter` and `WorkAccumulate` excerpts.
The separate family owners authenticate all geometry/force native leaves.
The existing test-only section/native mutexes protect their common blocks;
there is no production native linkage or new runtime context.

The oracle checks finite inputs/results, source-bound histories, interval
chronology and nonoverlapping output ranges before publication. It rejects
nonpositive running thickness under the same declared scope as the point
qualification. This wrapper is a selected prescribed-step source oracle,
not a complete OpenRadioss engine or an independently integrated trajectory.

## Decisive checks

The light host gate verifies physical thickness, nonzero elastic membrane and
bending response/work, exact source material binding, inconsistent sidecar
rejection, phase failure, unchanged output and retry. Native tests retain
independent QEPH and T3 point/family histories through loading, holding,
rotation beyond 1.6 rad and reversal, at two prescribed step durations. They
compare full native geometry/forces/couples, all point stresses, reported
thickness, native stiffness/timestep and all existing work channels. Wrong
endpoint-rate sampling and discarded prestress are detectable negative
controls. Last-point nonfinite input cannot publish a partial result.

The actual CUDA tests repeat the rotating elastic trajectory for each family,
compare against native histories, reject a wrong interval, retain output and
accepted bytes, and then retry against the independent native packet. The
owning CMake also reruns the existing J2 native rotating/plastic recurrence,
point/rate/analytic material checks, and optional existing J2 CUDA check.

Root-scheduled qualification (apply the workstation resource guard):

```sh
cmake -S lib_utest/qualification/shell_layered_law1_force -B BUILD_DIR \
  -DTL_LAYERED_LAW1_FORCE_NATIVE=ON -DTL_LAYERED_LAW1_FORCE_CUDA=ON \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_CUDA_ARCHITECTURES=120
cmake --build BUILD_DIR --parallel 1
ctest --test-dir BUILD_DIR --output-on-failure
```

The author-only 1-CPU/512-MiB gate uses
`TL_LAYERED_LAW1_FORCE_NATIVE=OFF`; its executable is
`shell_layered_law1_force_value_check`. Native/CUDA executables are
`shell_layered_law1_force_native_check` and
`shell_layered_law1_force_cuda_check`. Bazel owns
`//lib_utest/qualification/shell_layered_law1_force:shell_layered_law1_force_value_check`
and shares the existing header-only prescribed geometry fixtures through a
new test-only utility target.

This qualification does not establish element failure, membrane-only ELFORM9,
rigid-material part ownership, glass damage, arbitrary anisotropy/offsets,
self-contact, physical crash accuracy or a global energy residual tolerance.
