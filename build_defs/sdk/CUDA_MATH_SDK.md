# CUDA math SDK admission

The optional implicit FE demos use NVIDIA math libraries that are absent from
the compiler-only host CUDA installation. `cuda_math.bzl` admits an explicit
workspace-local SDK without modifying `/usr/local/cuda` or installing packages.
The exact downloads, archive hashes and licenses are recorded in
`cuda_math_manifest.json`; the extracted `sdk.json` is also pinned.

The qualified profile is Linux x86-64, CUDA 13.2.2 / nvcc 13.2.86, cuDSS 0.8.0.10,
and the retained Jitify revision `37b49d96bb41084dde895f0e7a2c3976db17b4e5`.
CUDA redistribution metadata and the cuDSS migration guide are authoritative:

- [CUDA 13.2.2 manifest](https://developer.download.nvidia.com/compute/cuda/redist/redistrib_13.2.2.json)
- [cuDSS 0.8.0 manifest](https://developer.download.nvidia.com/compute/cudss/redist/redistrib_0.8.0.json)
- [cuDSS API migration](https://docs.nvidia.com/cuda/cudss/migration_guide.html)

The repository rule requires these explicit environment values:

```text
--repo_env=CUDA_PATH=/usr/local/cuda
--repo_env=ROBODYNA_CUDA_MATH_ROOT=<absolute extracted SDK directory>
```

Also export the process variables `CUDA_PATH`, `CUDACXX` and
`CUDAToolkit_ROOT`: `.bazelrc` forwards them into compile actions. Repository
options alone do not set those action variables. The
[guarded CUDA demo command](../../examples/fea/cuda/README.md) shows both layers
with the current workspace SDK and cache paths.

The current workspace installation is `../crash-work/install/cuda-math-r0`.
The reusable provisioner is `tools/dependencies/cuda_math.py`. Download mode takes
`--manifest build_defs/sdk/cuda_math_manifest.json --downloads <directory>`.
Adding `--install <new directory>` verifies and extracts those cached archives.
Extraction is create-only and must use the shared workstation guard. Never mix
unverified archives or overwrite a previous installation/receipt after failure.

`@cuda_math` exposes cuBLAS, cuBLASLt, cuSPARSE, cuSOLVER, cuDSS, NVRTC and its exact
builtins companion, nvJitLink, Jitify and the existing toolkit's CCCL headers.
The `@rules_cuda//cuda:runtime` target is the single cudart owner. The driver stub
is an interface library for linking only; runtime `libcuda.so.1` comes from the
installed NVIDIA driver. Declared ELF dependencies and Bazel solib runpaths replace
ambient math-library flags; no global `LD_LIBRARY_PATH` workaround is required.

Small admission targets, run under the existing guarded build procedure:

```text
//build_defs/sdk:cuda_math_sdk_test
//build_defs/legacy/cudss:tests
```

The first target checks host library versions, shared-library loading and a tiny
NVRTC compilation including its builtins. The second checks matrix descriptor
types/pointers/layout, default reordering, and an exact inverse proof of the
retained Newton source. Neither target launches a GPU kernel or qualifies a
physics trajectory. The cuDSS adapter deliberately rejects an unreviewed header
version rather than silently admitting a later API.

DEME uses the same declared Jitify/NVRTC/CCCL SDK. Its owning CMake hook handles the
CUDA 13 CCCL include directory and preserves both runtime JIT include roots in a
generated header. It does not edit the dependency's source. Build this foreign
target, `@dem_engine//:dem_engine`, in a separate Bazel phase with `--jobs=1`: its nested CMake build has four
workers. Then build `//examples/fea/cuda:all_demos` with the normal four native
workers. Compilation and bounded per-demo runtime qualification remain distinct.
