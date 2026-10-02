# CPU Multicore dynamics

This target compiles the retained Multicore module: 27 original translation
units, with its existing rigid-body, particle/fluid, NSC/SMC and iterative solver
implementations. It uses the existing mixed mechanics backend and one selected
System clock. This is source integration, not a claim of independent MBD/FEA
libraries or a CUDA implementation of Multicore.

The explicit profile is `--config=multicore`. It sets the typed build capability,
enables OpenMP for host C++ compilation and linking, and passes OpenMP through
the separate nvcc host-options channel. Raw `-fopenmp` is not sent to nvcc's
device options. Every consumer sees the same generated `ChConfig.h`; enabling
the capability without OpenMP fails compilation instead of creating serial
`ChOMP` stubs behind a parallel module.

The core aggregate selects 15 additional original translation units: five custom
math sources and ten generic collision sources. Their owner is
`//src/collision/multicore:core_extension`, with an aggregate-header dependency
only. The public `:collision` facade includes the same mixed backend. There is
no second core library or duplicate `ChOpenMP.cpp` implementation. The existing
generic System factory is compiled with `CHRONO_COLLISION` so requesting
Multicore collision no longer takes its inherited Bullet fallback.

Precision remains double, SIMD remains disabled, and Thrust uses its CPU OpenMP
backend as specified by the retained Multicore CMake. Thrust backend definitions
are scoped to these owners and their Multicore consumers; unrelated CUDA FE/DEM
kernel owners keep their own backend selection. The default configuration does
not enable this feature and preserves the previously qualified header/owners.
The profile composes with `--config=multiphysics`. Previously qualified foreign
CMake and language-binding packages remain explicitly restricted to their
baseline profile until those extended configurations receive their own gates.

Build the 15 real inherited main programs with:

```text
bazel build --config=host --config=multicore //examples/mbd/multicore:native_demos
```

Use the declared SDK environment and shared workstation guard. The aggregate
does not run those programs: several select eight CPU workers, long horizons,
GUI windows or fixed output paths by default. Their runfiles/data layout and
individual runtime limits need separate admission before unattended execution.

Focused gates are `//src/mbd/multicore:tests`: exact CMake/source inventory,
real OpenMP team reporting through the existing `ChOMP` owner, generic collision
factory/contact/raycast behavior, SMC stepping, and the retained `real3` and
gravity tests. These small tests use at most two OpenMP workers and no display
or GPU context. Compiling a demo is not evidence that every solver or contact
option has passed runtime qualification.
