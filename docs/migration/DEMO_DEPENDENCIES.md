# Retained demo compilation: module and SDK plan

Source audit: 2026-10-01, starting from `e169095321`. This records build
requirements, not successful optional-module compilation or runtime validation.
The named source inventory remains the authority for individual paths:
[`CHRONO_DEMOS_TESTS_INVENTORY.json`](../verification/CHRONO_DEMOS_TESTS_INVENTORY.json).

There are 425 named demos in the standard tree: 311 C++, 97 Python and 17 C#.
One additional C++ FMI template demo brings the named total to 426. Helper
sources accompany these programs; they are not independent demo binaries.

## Existing owners and the shared build boundary

The root already builds the native core/mechanical-FEA aggregate, its extracted
neutral/visual owners, eight VSG translation units and a four-source CPU SCM
slice. These are reusable implementation libraries. The complete Vehicle,
robot-model, FSI, DEM, Multicore and other optional libraries need their own real
native targets before their demo targets can link.

Use one compiling owner for each translation unit. In particular, a full Vehicle
library must reuse the existing SCM sources, not compile a second copy alongside
the current terrain target. Do not link a second CMake-built core into executables
that already consume the native core. Retained CMake is the source of module
lists/options and a reference build, not an excuse for one opaque all-demo binary.

`build_defs/chrono/native_config.bzl` explicitly disables optional feature macros
for the qualified core profile. The shared `robodyna_cpp_demo` macro can enable
VSG after including that existing configuration: source review found no VSG or
Irrlicht guards affecting core mechanics layout. This presentation-only exception
must not be generalized to CUDA/OpenMP/precision flags without checking owning
headers and implementations. Each added feature must depend on its actual module
library, configuration headers and runtime data.

## Optional family matrix

Counts are named C++ / Python / C# demo source files, not runtime cases.

| Family | Counts | Required implementation/dependency work |
| --- | --- | --- |
| Vehicle | 64 / 21 / 11 | Full vehicle and vehicle-model libraries; separate VSG/Irrlicht adapters. Individual demos also need CRM/SPH, FMI, Multicore, OpenCRG or co-simulation support. |
| Robot | 17 / 6 / 0 | Robot models; most wheeled examples additionally require Vehicle. Industrial and Hexy demos retain Irrlicht dependencies. URDF-backed RoboSimian is an optional parser branch. |
| FSI | 24 / 1 / 0 | Generic three-source FSI interface; SPH library for 19 C++ demos; TDPF/HydroChrono for five. |
| Sensor | 16 / 6 / 1 | The local fork defaults to Vulkan RT, with an explicit shader compiler. OptiX and Apple Metal remain distinct profiles; some demo lists still select only particular backends. |
| ROS | 7 / 7 / 1 | Existing host-side library and protocol core plus a separate ROS 2 bridge executable. Compiling the host side alone does not provide the bridge. |
| SynChrono | 9 / 0 / 0 | Vehicle/models, MPI and FlatBuffers; DDS cases require the retained standalone FastDDS2.4.0/FastCDR1.0.24 recipe, kept separate from ROS middleware. |
| Multicore | 15 / 0 / 0 | OpenMP, Thrust and its own OMP backend configuration. |
| DEM | 5 / 0 / 0 | Three host sources, two existing CUDA sources and one optional VSG adapter. Native module batch is being prepared separately. |
| Cascade | 4 / 3 / 0 | OpenCASCADE SDK; selected Python cases additionally import `OCC`. |
| Modal | 4 / 0 / 0 | Spectra plus the native mechanics libraries. |
| Peridynamics | 5 / 0 / 0 | Existing FE backend, with Irrlicht/postprocess/MKL requirements by demo. Two source files lack active inherited CMake admission. |
| Parsers and YAML | 11 / 4 / 0 | Native parser/YAML owners, Python embedding and optional URDF/ROS libraries. |
| preCICE | 2 / 0 / 0 | Actual preCICE3 API, YAML parser profile, and SPH for the sphere participant; OpenFOAM peer runtime remains separate. |
| FMI | 4 / 0 / 0, plus one template | Pinned fmu-forge and native FMI integration. FMU export has a static-library requirement. |
| Postprocess | 7 / 2 / 0 | Two implementation sources; Gnuplot and renderer requirements depend on the example. |
| MUMPS | 1 / 0 / 0 | Existing MUMPS solver adapter and declared external MUMPS closure. |
| Irrlicht | 6 / 3 / 0 | Existing module plus a pinned Irrlicht SDK. |
| VSG | 8 / 1 / 0 | Reuse the existing native VSG owner and declared external SDK. |

Core (11 C++ / 4 Python / 1 C#), MBD (55 / 25 / 3), FEA (33 / 14 / 0)
and co-simulation (3 / 0 / 0) are tracked in the foundational demo batches.
Their optional branches must still name real dependencies rather than compile
empty preprocessor paths.

## Observed local SDK state

This is a scoped filesystem/PATH/package inspection, not a configure probe or a
claim that no other user environment contains a dependency.

- `/usr/local/cuda-13.2` contains NVCC, cudart and CCCL. CUDA 13 places the CUB and
  Thrust include roots under `include/cccl`; the older generated rules_cuda
  `cub`/`thrust` targets search the previous layout. Preserve the selected
  `@rules_cuda//cuda:runtime` owner and declare the actual CCCL include root.
- Existing workspace SDKs provide VSG 1.1.15, vsgXchange 1.1.12, vsgImGui 0.7.0,
  Vulkan headers/runtime, local SWIG and CPython 3.10 development support. Root
  Bazel already pins Eigen and GoogleTest. Compiler OpenMP headers are present.
- The inspected CUDA install lacks NVRTC and NPP libraries. A separate explicit
  CUDA math/runtime dependency batch is handling the required supplementary SDKs;
  a compiler-only CUDA installation is not a complete Sensor/math SDK.
- Irrlicht, OpenCASCADE, Spectra, MKL, MUMPS, URDF and ROS 2 development SDKs were
  not found in the inspected declared roots. No `dotnet`, `mono`, `mcs`, `csc`,
  `mpicxx` or `gfortran` was found on this session's PATH.
- MPI, HDF5, OpenVDB, GLEW and TinyXML runtime packages exist, but the corresponding
  development/compiler closure was not found. Runtime `.so` presence alone is
  insufficient for compilation.
- The five imported Chrono gitlinks remain empty. Their exact revisions are in
  [`DEPENDENCIES.json`](DEPENDENCIES.json): GoogleTest, Google Benchmark,
  FlatBuffers, fmu-forge and HydroChrono. Reuse root GoogleTest for native targets;
  obtain the other required recorded sources explicitly, without silently adopting
  differing historical local checkout revisions.

## SDK acquisition and qualification order

1. **Irrlicht:** obtain a versioned development package/source into the workspace;
   declare its headers, library and platform link/runtime closure in a local SDK
   repository, following the existing VSG/OpenSSL pattern. Build the retained
   Irrlicht module and one original demo. This unlocks many existing FE, MBD,
   robot and vehicle cases without rewriting their visualization setup.
2. **MUMPS:** provide headers and compatible MUMPS, BLAS/LAPACK and Fortran runtime
   libraries, including the chosen sequential/MPI variant. The inherited CMake
   adapter checks for a Fortran compiler and requests `MUMPS CONFIG`; do not assume
   a generic distribution runtime package includes that configuration. Native
   Bazel can declare the SDK closure directly; reference-CMake metadata must be
   supplied and verified separately. Keep the original solver selection in demos
   that specifically exercise MUMPS.
3. **MKL:** declare an actual oneMKL development SDK and its runtime dependencies.
   The retained Pardiso recipe uses `intel64`, dynamic linking, `intel_thread`
   and **LP64** defaults. Preserve the selected integer/threading ABI explicitly;
   do not mix ILP64 declarations or duplicate OpenMP runtimes accidentally. Build
   the one-source Pardiso adapter and its dependent cases after SDK admission.
4. **Other independent SDKs:** Spectra, OpenCASCADE, HDF5, URDFDOM/headers,
   console_bridge/TinyXML2, OpenCRG, MPI/Fast DDS and preCICE each get explicit
   providers and small owning-module gates. Their discovery must not silently
   disable a requested demo family.
5. **Sensor:** reuse the local fork's Vulkan design, but supply the required
   `glslangValidator` tool for its enabled GPU path. OptiX-specific cases also need
   OptiX and the CUDA/NPP/NVRTC closure. Do not substitute the CPU fallback and
   label it GPU compilation. Apple-only implementations remain platform-scoped.
6. **FMI/TDPF:** stage the recorded fmu-forge/HydroChrono sources and their real
   dependencies. TDPF requires HDF5. FMU export requires static native libraries;
   ordinary shared binding packaging is not the same profile.

## Python and C# are separate compilation products

Python syntax compilation is useful but insufficient. Package the original
scripts and helpers as real entry points, generate and compile the matching native
binding modules, and preserve one shared class-factory/core implementation across
them. Core and FEA runtime packages are qualified; the optional baseline package
is undergoing its import and cross-module ownership gates. The actual 97 demo
entry points require NumPy (15 programs), matplotlib (two), and pythonOCC
(three). Other dependencies found in auxiliary tools do not belong to that demo
denominator. Each required dependency must be declared in the selected Python
environment; source import discovery does not prove runtime qualification.

For C#, install/select a declared compiler/runtime and build the generated managed
assembly plus all required native wrappers. The retained demo CMake project uses
`check_language(CSharp)` and exits when it finds no compiler. Its separate wrapper
project defaults to `net472`, so a Linux .NET build needs a reviewed target/runtime
and reference-assembly plan. Existing native `libchrono.so` compilation and source
proxy comparison do not compile or execute the 17 managed demos. Keep native
wrapper ownership shared, including modules exchanging Body/Mesh/System objects.

## Execution sequence and acceptance

Expose programs individually through the common macro and group them in module
filegroups for build scheduling. No generated placeholder `main`, second solver
or replacement physics is needed. Preserve optional module arithmetic, precision,
stream and backend definitions. SPH intentionally keeps CUDA Thrust backend
definitions private; Multicore exports OMP Thrust definitions. Replicate those
boundaries instead of setting one global Thrust backend.

For each module batch: authenticate the source list/configuration, compile its
library, link every admitted original demo, then update the catalog with exact
targets and build receipts. Runtime qualification is a separate bounded step.
Interactive programs and GPU allocations are never executed merely by a compile
gate. SDK-blocked or inherited-unlisted cases retain explicit pending records;
an overall success flag must not conceal them. Use the existing workstation guard
and compiler-worker limits, with target-local limits for large CUDA/wrapper units.
