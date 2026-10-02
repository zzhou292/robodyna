# preCICE adapters

Three original implementation files are owned by `:base`, `:mbs` and `:sph`.
They reuse the current mechanics, YAML parser and SPH owners. No original C++
algorithm or model input is changed. `SourceBaseline.json` authenticates all
three implementations, their headers, three original mains and37 runtime assets.

Build `//examples/coupling/precice:all_demos` with `--config=yaml
--config=fsi-sph`. The sphere example needs SPH at compile time because its
original function accesses the SPH participant outside its conditional guard.
The flap/OpenFOAM example and `//apps/precice:run_adapter` need only
`--config=yaml`. The latter preserves the existing YAML application outside the
demo source tree. The current `yaml` profile already enables VSG coherently for
the producer and consumer public types. This matrix does not qualify a separate
headless YAML profile; never inject a demo-only VSG flag into those types.

The SDK is the official preCICE3.0 Jammy package with its original PETSc/MPI
dependencies. Its provider authenticates44 preCICE/PETSc runtime DSOs, reuses five
DSOs owned by `@mpi_sdk`, and pins68 recursive platform libraries. No system
installation is modified. This is an explicitly local Ubuntu22.04/Linux x86_64
profile with inherited absolute RPATH; it is not a relocatable binary release.

The host test checks the actual loaded preCICE library identity/version, invokes
its configuration validator, verifies that the original MBS adapter retains one
provided System, and loads the original sphere model and coupling declarations.
The validator initializes MPI internally. Its gate needs the declared OpenMPI
executable, help and plugin layout through the existing single-rank launcher;
linking `libmpi` alone is insufficient. It does not initialize paired solver
participants, create a renderer, construct SPH or qualify a coupled trajectory.
Runtime qualification requires that complete guarded validator test to pass.

Full runs additionally require writable, create-only run directories, correctly
resolved YAML/XML/geometry paths and an actual peer participant. The flap case
requires OpenFOAM and its preCICE adapter, which this SDK does not supply. The
sphere's original `Fluid_BUOY` branch is a simplified buoyancy/drag example, not
a CFD solver; its distinct SPH branch uses the real existing CUDA SPH owner.
The retained YAML defaults request HDF5 output; baseline HDF5 capability and
output behavior must be qualified separately before promising those files.

Known inherited partial implementations remain visible: the MBS FE-mesh
registration and parts of the SPH coupling/checkpoint support contain TODOs.
Source retention or successful compilation does not establish those behaviors.
