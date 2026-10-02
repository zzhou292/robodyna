# Retained solver-independent FSI coupling

`//src/coupling/fsi:fsi` owns the three original `ChFsiSystem`,
`ChFsiFluidSystem` and `ChFsiInterface` implementation files. It consumes the
existing mechanical core and its actual rigid/FE participants. "Generic" here
means independent of a particular fluid solver; this is not a claim of an
independent neutral-mechanics or fluid-only library.

The existing coupling driver, CFD/MBD substeps, concurrent MBD advance, join,
force/state exchange order and time bookkeeping are unchanged. This build batch
does not introduce another nodal owner, call a second system step externally,
or replace the existing partitioned coupling with another method.

SPH depends on this interface through `//src/sph:sph`. The future TDPF owner can
reuse it when the pinned HydroChrono source and HDF5 profile are admitted. No
placeholder potential-flow implementation is provided.
