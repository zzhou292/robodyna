# Temporary Chrono host bridge

`//build_defs/chrono:host_core_fea` builds the absorbed first-party source at
`src/compatibility/chrono` with a declared Bazel `rules_foreign_cc` CMake action.
It does not link the old workspace installation or access the original checkout.
This is an interim aggregate, **not independent FEA and MBD libraries**.

The initial profile is Linux, static core plus mechanical FEA, GPU vendor `NONE`,
one-thread runtime tests, no OpenMP/SIMD, and no optional Chrono modules. Eigen is
the root module's pinned Bazel dependency; the input hook rejects missing staged
headers. Generated configuration/version headers and installed headers are action
outputs. CMake/toolchain actions are declared by `rules_foreign_cc`; ordinary host
compiler/system-library prerequisites still apply. This is not a claim of full
cross-platform or hermetic toolchain qualification.

The foreign action uses at most four nested compiler workers. Run it under the
workspace heavy-build resource guard and permit only one heavy build at a time;
`bazel --jobs` by itself does not cap nested CMake workers. Nothing in this package
starts a build automatically during import.

After source imports and root dependency/toolchain admission, the intended checks
are:

```sh
bazel build --jobs=1 //build_defs/chrono:host_core_fea
bazel test --jobs=1 --local_test_jobs=1 //tests/chrono:host_bridge_tests
```

These commands are test entry points, not a record of having executed them. The
tests check frame transformations, JSON serialization/factory registration, free
rigid translation and a small FE oscillator against its analytic solution. They
exercise the inherited implementation from the new build; they do not requalify
all mechanics or prove standalone domain linkage. Static registrations are kept
with `alwayslink` until native packages provide an explicit registration strategy.

VSG is intentionally absent from this first target. Its future target must declare
VSG, vsgXchange, vsgImGui and Vulkan/shader dependencies and run the existing accepted
Yaris replay checks. Do not discover the old installation through host paths or
pretend that source retention means rendering is already built in Bazel. Likewise,
thermal FEA, SPH, vehicle, sensors and other retained modules get their own explicit
profiles and gates rather than enabling every dependency during this first build.

Native extraction proceeds through neutral math/serialization, numerical and
mechanics contracts; then independent `fea` and `mbd` targets; then explicit mixed
coupling targets. Preserve node-to-frame and monolithic constraint semantics.
Retire this bridge only after independent domain and combined-system tests pass.
