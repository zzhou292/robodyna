# Retained managed examples

`DEMO_TARGETS.json` inventories all 17 original C# examples, each with a real
assembly target. All 17 assemblies have passed compilation: the core example,
three MBD examples, nine baseline Vehicle examples, two OpenCRG examples,
Sensor and ROS Sensor. Baseline and optional runtime/ownership checks also pass,
and the final complete matrix passed on the settled source snapshot.
Inventory entries are not placeholder build targets.

With the declared Mono and existing binding SDK inputs admitted, use the normal
workstation guard and the two-worker native-wrapper trial described in
[`build_defs/bindings/README.md`](../../build_defs/bindings/README.md):

```sh
bazel build //examples/csharp:build_system
bazel test //examples/csharp:build_system_test
bazel run //examples/csharp:build_system -- --output /absolute/path/to/new-output
```

The output directory is create-only. The original demo writes its JSON archive
there and prints actual crank positions during its 5-second headless simulation.
The launcher records stdout, resolved runtime inputs and the observed native
backend. Running a build does not start the simulation. The outer workstation
watchdog remains the owner of CPU/RAM/process limits.

Baseline wrappers retain their baseline ABI guards. OpenCRG, OptiX and ROS-Sensor
routes use distinct coherent profiles. Extra FE fields, Multicore and TDPF remain
excluded from these managed profiles. Actual optional binding generation, native
linkage and cross-module object ownership have separate qualification gates.

The ROS Sensor composition includes Robot bindings. Its RoboSimian geometry
helpers are named `RoboSimianBoxShape`, `RoboSimianCylinderShape` and
`RoboSimianSphereShape` in C#, because they are different native types from the
Core `BoxShape`, `CylinderShape` and `SphereShape`. Core, Python and C++ names
remain unchanged. Equivalent generated pointer/vector helpers retain their
reviewed first owner through exact hash-pair approvals; unlike native types are
never merged based on a shared filename.

The first managed compilation and headless execution gate passed:
`crash-work/reports/robodyna-managed-core-build-1.json`, 18.873 seconds under the
guard. It includes the three input-admission cases, actual process-creation
failure/reuse handling, and the original core demo: 501 steps to 5.01 seconds,
finite crank motion across a 2.00098-metre range, and one observed native backend.
The later `robodyna-managed-optional-build-3.json` gate passed all 13 assemblies
and six tests (32.244 seconds under the guard). It uses Roslyn 3.11/C# 7.3 with
Mono 6.8/net472 and the declared XML LINQ runtime overlay. Original local-function
syntax is preserved. Native ELF ownership and actual cross-module managed
calls passed: core time advance, Vehicle terrain/tire values, postprocess System
sharing, VSG shared-pointer casts and one loaded owner per implementation.
No GUI window or CUDA device was initialized by that runtime coupon.

Build the compiled optional groups with
`//examples/csharp/mbd:assemblies` and `//examples/csharp/vehicle:assemblies`.
These are real original Main assemblies. Unlike the core `build_system` preset,
the other 16 assembly targets do not yet provide `bazel run` launchers. Compiling
them does not qualify their interactive trajectories or replace explicit
launch/data/resource admission.
