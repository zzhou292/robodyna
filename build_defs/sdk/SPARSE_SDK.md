# CPU sparse solver SDK profiles

The native interfaces reuse the retained implementations: two MUMPS translation
units and one PardisoMKL translation unit. They link the actual vendor solvers;
no iterative or approximate solver is substituted to make a demo compile.

| Repository | Environment root | Admitted profile |
| --- | --- | --- |
| `@mumps_sdk` | `ROBODYNA_MUMPS_ROOT` points to the extracted `usr/` directory | Ubuntu 22.04 MUMPS 5.4.1-2, sequential MPI stubs, 32-bit `MUMPS_INT`, Scotch 6.1.3-1 |
| `@mkl_sdk` | `ROBODYNA_ONEAPI_2023_ROOT` points to extracted `opt/intel/oneapi/` | Intel oneMKL 2023.2.0, dynamic LP64, Intel OpenMP 2023.2.0 |

`sparse_pins.json` authenticates the selected headers and vendor shared libraries.
Each repository emits `sdk.json` recording the supplied root, exact libraries,
SONAME/dependency evidence and platform runtime prerequisites. No old Chrono
implementation library is imported. Workspace extraction uses `dpkg-deb --extract`,
without running package scripts, installers, `ldconfig` or global package changes.

MUMPS uses the matching `libmumps-seq-5.4`, `libmumps-seq-dev` and
`libmumps-headers-dev` packages. Its 5.4 SONAMEs and 5.4.0 package filenames are
consistent with the package's 5.4.1 headers. The regular parallel-MPI MUMPS library
is not interchangeable with this profile. BLAS, LAPACK and the recorded Fortran
and compression runtime dependencies remain explicit OS prerequisites. A local
Fortran compiler is unnecessary for linking these prebuilt libraries to the
unchanged C++ interface.

The retained installation guidance explicitly identifies Intel 2023 as compatible
with this Eigen interface and warns against Intel 2025. The selected packages
come from Intel's official oneAPI repository: MKL runtime, common development
headers, common files/licensing, and the small CPU OpenMP runtime package.
SYCL/compiler/TBB components are not part of this CPU profile. CPU dispatch DSOs
are retained as runtime data. The SDK-derived dynamic search directories follow
the vendor CMake contract; relocating this local SDK requires rebuilding.
This is not yet a portable binary installer.

MKL also locates CPU dispatch libraries beside its loaded core using the loader's
path. The initial real solve exposed that Bazel's default per-import `_solib`
directories separate these siblings. The MKL imports therefore use each exact
vendor DSO as its link interface, with `system_provided=True`; the declared local
SDK RPATH supplies the runtime SONAMEs in their original common directory.
This attribute does not search for a global MKL installation: roots, hashes and
runtime data remain explicit. The runtime test examines the actual loaded path
without resolving symlinks, requires all13 pinned companions there, and executes
a real BLAS operation before this packaging can be called qualified.

The module keeps `EIGEN_USE_MKL_ALL` and `MKL_LP64` private, as its owning CMake
does. The qualified mechanics core's flags remain unchanged. The public native
labels are `//src/numerics/sparse:mumps` and `:pardiso_mkl`; both currently retain
the explicit mixed-backend dependency.

Qualification targets are `//tests/sparse:mumps_test` and
`//tests/sparse:pardiso_test`. They check integer width, real sparse solves and a
real coupled FE/body trajectory with the selected solver. Pardiso also exercises
the complex interface. Tests run in separate processes with one requested CPU
worker so vendor BLAS closures are not accidentally combined during admission.
No GPU or MPI service is launched. Compilation and actual numerical results must
pass before marking the SDK profile qualified.

The source-only declaration of a SDK does not prove CUDA execution, all optional
solver modes, or runtime portability to another machine. Existing screenshots,
archives and solver-comparison evidence remain separate from this admission.
