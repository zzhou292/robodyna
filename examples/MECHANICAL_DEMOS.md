# Native mechanical demo build map

`MECHANICAL_DEMOS.json` maps every original CMake-listed core, multibody and FEA
program to a descriptive Robodyna label. The current batch declares 99 real
executables from their original `main` sources, with private headers and literal
data dependencies. Five optional programs require coherent Multicore or field
FEA configurations. Their declared targets compile the original source entry
points and fail when the required configuration is absent.

| Package | Retained programs | Current native declarations |
| --- | ---: | ---: |
| `//examples/core` | 11 | 11 |
| `//examples/mbd` | 55 | 55 |
| `//examples/fea` | 33 | 33 |

The first 48 core/VSG declarations passed compilation in
`crash-work/reports/robodyna-mechanical-demos-build-2.json`. The additional 30
Irrlicht declarations passed compilation. See `SUPPORT_DEMOS.json` for the additional
socket, visualization and postprocess programs outside these three families.
The next16 declarations use the real pinned MUMPS/Pardiso providers; their
compilation and sparse-solve tests pass. Qualification of the five optional
configuration cases remains in progress.

Compile the current slice explicitly:

```
bazel build //examples/core:native_demos //examples/mbd:native_demos //examples/fea:native_demos
```

The complete MBD group is `//examples/mbd:all_demos` with `--config=multicore`.
The complete FEA group is `//examples/fea:all_demos` with `--config=multiphysics`.
Supply the explicitly selected SDK roots documented in the dependency plan.

Use the workspace's bounded build guard for workstation qualification. These
targets do not launch simulations during a build. The shared macro links the
existing native CPU mechanics aggregate and, where selected, the real VSG
module. Its demo-only feature header advertises only that linked visualization
module after including the existing core configuration. Numerical compilation
flags and the core's feature profile remain unchanged.

The existing qualified `//examples/mbd:spring`, `//examples/mbd:collision_nsc` and
`//examples/scm:rigid_tire` capture workflows are retained. New
`spring_interactive` and `collision_nsc_interactive` targets compile the original
interactive branches independently of those bounded capture hooks.

`euler_beams` uses the unchanged source's MINRES solver. Although the old CMake
file grouped it under the MKL option, its actual source has no MKL dependency.
Programs that instantiate optional solvers retain those requirements. The
unconditional but unused MKL include in `ancf_contact_smc` is still present;
the declared Pardiso provider supplies its header while the demo continues to
select its original MINRES solver.

This is a **compile-first** batch. Successful compilation does not qualify
interactive execution, optional backend behavior or GPU mechanics. The inherited
data path defaults to `../data/`; shared runfiles and writable-output admission
must be completed before advertising the new targets as ready to run. Literal
asset inputs in the map are useful build dependencies, not proof that all
runtime-selected inputs have been admitted. Existing capture profiles retain
their separate tested resource and data-path controls.
