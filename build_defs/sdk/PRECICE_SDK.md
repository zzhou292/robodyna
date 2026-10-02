# Local preCICE SDK

Set `ROBODYNA_PRECICE_ROOT` to the admitted `precice-r0` extraction root and
`ROBODYNA_PRECICE_RUNTIME_ROOT` to the separate36-package PETSc/Boost runtime
overlay. Keep `ROBODYNA_MPI_ROOT` pointed at the already-admitted OpenMPI `usr`
directory. `@precice_sdk//:precice` reuses the same C/C++/Fortran MPI owners;
it never imports another MPI installation or a mechanics implementation.

The provider authenticates the official preCICE3.0 package receipt, nine public
headers, runtime extraction receipt, ELF SONAME/NEEDED/hash records and recursive
platform prerequisites. The initial read-only loader scan reported no unresolved
dependency. Actual linking/configuration execution remains a separate native gate.
Receipts and licenses remain at their original extraction roots; the preCICE
license and declared SDK runtime files are included in runfiles.

This transitional Linux x86_64 profile uses explicit absolute DT_RPATH directories
because the packaged PETSc dependency chain has no own RUNPATH. Platform runtime
updates require renewed pin admission. No global loader environment, system
package installation or solver substitution is performed. Portable distribution
needs its own relocated-runtime and license-packaging qualification.
