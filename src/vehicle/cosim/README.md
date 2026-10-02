# Vehicle co-simulation

This package compiles the original MPI force/displacement co-simulation code.
It reuses the existing Vehicle, Robot, SCM, Multicore, SPH and DEM implementations;
it does not add another numerical backend or replace their algorithms.

Twenty retained C++ files have nine explicit compilation owners. `base` owns the
six framework/node-interface files; `mbs`, `robot`, `rigid_tires`,
`flexible_tires`, `cpu_terrain`, `omp_terrain`, `sph_terrain` and `dem_terrain` own
their original concrete families. `cosim` composes the baseline and any enabled
Multicore/SPH profile. `dem_terrain` is separately available for explicit CUDA
composition. Source inventory and compile-owner tests check unique ownership.

The MPI model intentionally advances an MBS subsystem, tire subsystems and
terrain subsystems on different ranks, exchanging the original lagged interface
data. Each rank links one mechanics implementation. This is the retained
partitioned method, not a single global monolithic solve.

## Configuration and demo coverage

`//examples/vehicle/cosim:baseline_demos` contains six original mains. The other
two original mains are `wheeled_vehicle_omp` and `wheeled_vehicle_sph`, requiring
`--config=multicore` and `--config=fsi-sph`, respectively. `all_demos` includes all
eight and rejects a missing required profile. The catalog records exact required
MPI rank counts and current qualification status.

Enable graphics with the build-wide `--config=vsg`, which configures both node
implementations and consumers. A demo-only VSG define would change public class
layouts and is not valid here. Postprocessing is consistently disabled in this
first profile because `CHRONO_POSTPROCESS` also changes the base node layout.
Its later admission must configure every producer and consumer together.

The first profile preserves the retained no-Pardiso branch: flexible tires use
SparseQR and HHT. Original demos that request Pardiso through `SetChronoSolver`
report their existing SparseQR fallback. This profile does not claim Pardiso
execution. No solver selection code or physical parameter was changed.

Native compilation and source tests are pending the shared guarded build.
Declaring a demo executable does not establish runtime readiness: JSON inputs,
referenced geometry, output directories, MPI rank counts and per-rank resource
budgets must be admitted before launching a full example. Original mains remain
unchanged, including their duration and GUI choices.

## Small runtime gate

`//src/vehicle/cosim:mpi_transport_test` launches four local CPU ranks using the
declared OpenMPI SDK and the existing owned-session watchdog. It exercises:

- Rejection of an insufficient rank count, then actual framework initialization.
- A two-rank terrain communicator, its ordering and a real collective.
- The original all-rank MBS/tire/terrain census.
- Empty geometry and exact materials, primitives, vertices, normals and triangle
  connectivity exchanged through the original send/receive implementation.

No dynamics, CUDA context or renderer is created in this coupon. A pass proves
that bounded transport path, not stable distributed vehicle physics or throughput.
The follow-up physical gate should use the existing rig/tire/terrain setup with
an explicit short duration, captured interface forces and subsystem clocks.

## Inherited boundaries retained for separate repair

`ChVehicleCosimBaseNode.cpp` currently sends mesh arrays specifically to
`TERRAIN_NODE_RANK`, even though the utility accepts a destination argument.
The coupon uses the supported tire-to-terrain route. It does not claim arbitrary
mesh routing. Sender mesh scratch arrays and the initialization census array
lack deletion in the retained source; the coupon performs bounded one-time
initialization/exchange and is not a leak-free or long-run qualification. Convex
hull transport is an explicit original TODO and is not advertised by this gate.
These defects are recorded without silently changing preserved source bytes.
