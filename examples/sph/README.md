# Retained FSI/SPH examples

`//examples/sph:native_demos` groups 19 original SPH main programs as individual
native binaries. Each links the actual SPH/coupling libraries and VSG adapter.
Four flexible-FEA examples additionally select the existing real PardisoMKL
library, matching their original CMake admission; no alternative solver is
substituted merely to increase the compilation count.

[`DemoCatalog.json`](DemoCatalog.json) preserves all 24 original C++ FSI cases.
The other five TDPF demos have real native targets under
`//examples/coupling/tdpf`, using the pinned HydroChrono and HDF5 owners.
All 24 programs passed the current full compile matrix. Python FSI scripts are
a separate binding/package admission.

The initial SPH build keeps its original float default, CUDA equations, parameter
defaults, CPU/GPU substep policy and data-exchange ordering. These are build
declarations. The catalog preserves its initial `compiled` and
`runtime_qualified` fields; current compilation evidence is recorded separately
in the matrix's per-target receipts. It does not qualify a fluid trajectory.

The programs preserve their original CLI, model-data and output conventions.
Some use meshes, BCE CSV data, JSON settings or optional reconstruction tools;
declare their complete input closure before execution. VSG assets alone are not
a fluid-model data pack. Configure the data path before renderer construction
and use a fresh writable output directory. Runs need per-case resource/duration
guards; compilation must never implicitly execute large particle simulations.

Focused integration gates are `//src/sph:source_inventory_test`,
`:ownership_test` and `:host_profile_test`. Real SPH tests such as Poiseuille flow,
domain guards and rheology remain retained source and require their own CUDA
runtime qualification. Building an example is not proof of those physical
results. See the [solver ownership contract](../../src/sph/README.md).
