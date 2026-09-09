# Isolated VSG dependencies for the first replay

The sources and Linux development headers below are staged and SHA-256 checked. **glslang, VSG, vsgXchange, vsgImGui and isolated Chrono core+VSG are built and installed locally; the external replay executable links successfully.** All three owning CPU lifecycle tests and the actual RTX 5090 R0/R1 window/PNG gates passed. [Runtime evidence](../../crash-work/reports/vsg-r0-r1-runtime-checkpoint-1.json) covers 15 normal-rig and 81 final coupon frames, including independent source-stamp/hash checks and visual review. This supports [the rendering architecture](../docs/RENDERING_ARCHITECTURE.md) without adding mechanics capability.

[dependencies.lock.json](dependencies.lock.json) records exact commits, official URLs, archive sizes/hashes, license-file hashes, Ubuntu package hashes, and local runtime forwarding links. Large source trees stay under `crash-work/dependencies/vsg-r0`, outside application Git. Six source archives plus six development packages total **9,626,302 bytes**. The guarded staging peaked at **46,399,488 sampled RSS bytes**, one CPU; no GPU was requested. Sandbox DNS and GitHub API rate-limit failures are retained alongside successful bounded retries in `crash-work/reports/vsg-*.json`.

The lock is the unchanged source-staging snapshot. The [first build checkpoint](../../crash-work/reports/vsg-r0-build-checkpoint-1.json) records the initial glslang/VSG stage. The [current build checkpoint](../../crash-work/reports/vsg-r0-build-checkpoint-2.json) records installed libraries, source/binary hashes, resolved runtime links, the three-test XML and every retained report. Measured builds used one job and one affinity CPU:

| Build | Elapsed time | Peak sampled process-tree RSS |
|---|---:|---:|
| glslang | 58.9 s | 460 MiB |
| VSG | 150.9 s | 750 MiB |
| corrected vsgXchange | 27.3 s | 472 MiB |
| vsgImGui | 30.1 s | 580 MiB |
| fresh Chrono core+VSG and owning tests | 240.2 s + 107.8 s | 966 MiB |
| external replay compile/link retry | 3.5 s | 773 MiB |

Chrono's first batch stopped at the planned 240-second timer with useful objects retained; the second completed without a compiler or resource failure. The first viewer compile caught and corrected `CameraVerticalDir`'s namespace; both logs remain available. VSG's generated header enables shader compilation and XCB windowing, disables the optimizer, and its link line resolves the three pinned glslang libraries. The executable's `ldd` check resolves Chrono/VSG/glslang to the isolated prefixes and Vulkan/XCB to existing host runtimes, with no missing libraries. This verifies compilation/linking and CPU configuration behavior, not shader execution or device compatibility.

## Source and license inventory

| Dependency | Chrono pin / exact revision | Retained license |
|---|---|---|
| VulkanSceneGraph | v1.1.15 / `599a8c5c61cbb993079261b2ada99bd843badf5b` | MIT; bundled Vulkan definitions have a separate retained notice |
| vsgXchange | v1.1.12 / `63d27da61a7b0d82dcfe71a13bb1b17f03b9d125` | MIT; bundled STB, TinyGLTF, JSON and image headers retain their own notices |
| vsgImGui | v0.7.0 / `37ef59ffa78482ee50b96c17b03bf3e0fbf0f973` | MIT |
| glslang | 16.1.0 / `b5782e52ee2f7b3e40bb9c80d15b47016e008bc9` | Mixed source licenses; retain the complete `LICENSE.txt` and per-file notices, including preprocessing terms |
| Dear ImGui | parent gitlink `993fa347495860ed44b83574254ef2a317d0c14f` | MIT, with bundled STB notices |
| ImPlot | parent gitlink `f156599faefe316f7dd20fe6c783bf87c8bb6fd9` | MIT |

The first four pins come from [Chrono's Linux helper](../../chrono/contrib/build-scripts/linux/buildVSG.sh). The GUI child commits were verified from the exact parent Git tree and assembled into its otherwise-empty `src/imgui` and `src/implot` directories. Original archives remain unchanged. This prevents the pinned vsgImGui CMake file from trying `git submodule update` during configuration. This is a source notice inventory, not a substituted blanket license for bundled code.

The helper itself must not be executed unchanged: it deletes the install prefix, creates unrestricted build jobs, edits shell startup files, builds extra packages, and invokes glslang's external-source updater. Our build needs none of those side effects.

The pinned vsgXchange disabled-Assimp fallback failed compilation because `src/assimp/assimp_fallback.cpp` includes `vsgXchange/models.h` after the class declaration moved to `vsgXchange/assimp.h`. A separate source overlay changes exactly that include; Assimp remains disabled. The [patch manifest](../../crash-work/dependencies/vsg-r0/patches/assimp-header-manifest.json), [one-line patch](../../crash-work/dependencies/vsg-r0/patches/assimp-header.patch) and [SHA-guarded preparation helper](../../crash-work/dependencies/vsg-r0/patches/prepare_assimp_header.py) preserve attribution and provenance. The original file SHA-256 is `4411115b4c38d00711da84b64a0d677312000de62dafe1f61519dc7e8433ed51`; the corrected file is `b657d1966c78116d86babc6ec95c964ec7f8df28e3cf70e49625ac10aa1e752c`. Original source, archive and MIT notice remain unchanged; the failed build report is retained as `vsg-xchange-build-1.json`. The corrected build and install passed.

## Linux closure already staged

Ubuntu development packages were downloaded from the official archive, checked against SHA-256 and size from the existing local APT metadata, then extracted with `dpkg-deb -x` into `ubuntu-dev/sysroot`. No package was installed and no maintainer scripts ran:

| Package | Exact version | Purpose |
|---|---|---|
| libvulkan-dev | 1.3.204.1-2 | Vulkan headers, linker/pkg-config metadata |
| libxcb1-dev | 1.14-3ubuntu3 | VSG's Linux window backend |
| libxau-dev | 1:1.0.9-1build5 | XCB's private pkg-config/header closure |
| libxdmcp-dev | 1:1.1.3-0ubuntu5 | XCB's private pkg-config/header closure |
| libpthread-stubs0-dev | 0.4-1build2 | Development metadata |
| x11proto-dev | 2021.5-1 | X protocol headers |

Matching Vulkan, XCB, Xau and Xdmcp runtimes already exist on the host; workspace-only symlinks resolve the extracted development links to their exact installed files. The lock records those files' hashes. This is a build-header closure, not a complete standalone OS package environment; protocol documentation tools are unnecessary. The package copyright files are retained, including Apache-2.0/MIT Vulkan and X11/MIT notices.

The pinned VSG requires Vulkan **>=1.1.70.0**, and its Linux implementation requires **xcb only**. Local pkg-config confirms Vulkan 1.3.204 and XCB 1.14. No Xlib, XRandR, xcb-randr, xkbcommon, full LunarG SDK, new loader, or driver installation is needed for this source closure. Actual device/window compatibility was checked separately by the retained RTX 5090 runs below.

## Configure/build sequence for the coordinator

Use one build job inside the shared workstation guard; never overlap this build or viewer with a mechanics GPU run. Start with an **8 GiB job-RSS cap**, **32 GiB free-RAM reserve**, one or two affinity CPUs and one worker. Estimates before measurement are roughly 1–3 GiB for individual dependency compilations and 2–8 GiB for Chrono's large VSG translation units; these are capacity allowances, not measured requirements. Start with Release, shared libraries, no LTO or debug duplicate, and no glslang PCH. If a cap trips, inspect the retained report before changing it. Reserve roughly 2 GiB disk for sources/build/install until measured.

The installed CMake 3.22.1 meets the pinned packages' minima. Ninja is absent; `Unix Makefiles` avoids another dependency. In a command-local shell, define:

```bash
VSG_WORK=/home/jsonzhou/Desktop/chrono-work
VSG_STAGE="$VSG_WORK/crash-work/dependencies/vsg-r0"
VSG_PREFIX="$VSG_WORK/crash-work/install/vsg-r0"
VSG_BUILD="$VSG_WORK/crash-work/build/vsg-r0"
VSG_DEV="$VSG_STAGE/ubuntu-dev/sysroot"
export PKG_CONFIG_SYSROOT_DIR="$VSG_DEV"
export PKG_CONFIG_LIBDIR="$VSG_DEV/usr/lib/x86_64-linux-gnu/pkgconfig:$VSG_DEV/usr/share/pkgconfig"
export PKG_CONFIG_ALLOW_SYSTEM_CFLAGS=1
export PKG_CONFIG_ALLOW_SYSTEM_LIBS=1
VSG_COMMON=(-G 'Unix Makefiles' -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=ON
  -DCMAKE_INSTALL_PREFIX="$VSG_PREFIX" -DCMAKE_INSTALL_LIBDIR=lib
  -DCMAKE_PREFIX_PATH="$VSG_PREFIX" -DCMAKE_INSTALL_RPATH="$VSG_PREFIX/lib"
  -DCMAKE_INTERPROCEDURAL_OPTIMIZATION=OFF
  -DVulkan_INCLUDE_DIR="$VSG_DEV/usr/include"
  -DVulkan_LIBRARY="$VSG_DEV/usr/lib/x86_64-linux-gnu/libvulkan.so")
```

Both `PKG_CONFIG_ALLOW_SYSTEM_*` settings matter: without them, this workstation's pkg-config strips `/usr` flags before sysroot translation and returns only `-lvulkan -lxcb`. With them, the checked flags contain the isolated include and library directories. Keep this environment available to every downstream configure that consumes VSG's exported package; no `.bashrc` change is needed.

Configure, build, and install **each stage before configuring its consumer**. Each command must be separately wrapped by `run_bounded.py` with the shared `crash-work/reports/workstation.lock`; the following are the inner configure commands, not instructions to run them concurrently:

```bash
cmake -S "$VSG_STAGE/glslang/source/glslang-b5782e52ee2f7b3e40bb9c80d15b47016e008bc9" -B "$VSG_BUILD/glslang" "${VSG_COMMON[@]}" \
  -DENABLE_OPT=OFF -DBUILD_EXTERNAL=OFF -DGLSLANG_TESTS=OFF \
  -DENABLE_GLSLANG_BINARIES=OFF -DENABLE_PCH=OFF -DENABLE_SPIRV=ON -DGLSLANG_ENABLE_INSTALL=ON

cmake -S "$VSG_STAGE/vsg/source/VulkanSceneGraph-599a8c5c61cbb993079261b2ada99bd843badf5b" -B "$VSG_BUILD/vsg" "${VSG_COMMON[@]}" \
  -DVSG_SUPPORTS_ShaderCompiler=ON -DVSG_SUPPORTS_ShaderOptimizer=OFF -DVSG_SUPPORTS_Windowing=ON

cmake -S "$VSG_STAGE/patched/vsgXchange-assimp-header" -B "$VSG_BUILD/vsgXchange-assimp-header" "${VSG_COMMON[@]}" \
  -DCMAKE_DISABLE_FIND_PACKAGE_osg2vsg=ON -DCMAKE_DISABLE_FIND_PACKAGE_draco=ON \
  -DCMAKE_DISABLE_FIND_PACKAGE_Freetype=ON -DCMAKE_DISABLE_FIND_PACKAGE_assimp=ON \
  -DCMAKE_DISABLE_FIND_PACKAGE_Ktx=ON -DCMAKE_DISABLE_FIND_PACKAGE_GDAL=ON \
  -DCMAKE_DISABLE_FIND_PACKAGE_CURL=ON -DCMAKE_DISABLE_FIND_PACKAGE_OpenEXR=ON

cmake -S "$VSG_STAGE/vsgImGui/source/vsgImGui-37ef59ffa78482ee50b96c17b03bf3e0fbf0f973" -B "$VSG_BUILD/vsgImGui" "${VSG_COMMON[@]}" \
  -DSHOW_DEMO_WINDOW=OFF
```

For each selected stage, the guarded inner build is `cmake --build "$VSG_BUILD/STAGE" --parallel 1`, followed by the guarded `cmake --install "$VSG_BUILD/STAGE"`. Use fresh versioned output directories; do not delete an existing prefix. vsgXchange includes its small `vsgconv` executable unconditionally; it does not require the separate vsgExamples repository.

At the VSG configure gate, verify generated `vsg/core/Version.h` has `VSG_SUPPORTS_ShaderCompiler` enabled and the link targets include `glslang::glslang`, `glslang::glslang-default-resource-limits` and `glslang::SPIRV`. The pinned CMake warns and disables compilation if discovery fails even when requested ON; a successful configure exit alone is insufficient. `ENABLE_OPT=OFF` and `BUILD_EXTERNAL=OFF` avoid SPIRV-Tools and the glslang updater without removing SPIR-V generation.

Next configure an isolated Chrono **core+VSG** build with `CH_ENABLE_MODULE_VSG=ON`, shared libraries, `BUILD_TESTING=ON`, `BUILD_TESTING_BASE=OFF`, `BUILD_TESTING_VSG=ON`, demos and unrelated modules OFF, the same Vulkan/pkg-config environment and prefix. Build `Chrono_vsg` and `utest_VSG_LoadingThreads` with one job in bounded, resumable batches. Keep the existing mechanics core build untouched. Link the external viewer using Chrono's package/export pattern and the actual exported `Chrono::Chrono_vsg` target (`Chrono::vsg` is an in-tree alias only). Confirm generated config has VSG enabled; inspect runtime link resolution and shader compilation before admitting replay.

The completed local Chrono build/install paths are `crash-work/build/chrono-vsg-r0` and `crash-work/install/chrono-vsg-r0`. The external build uses `Chrono_DIR=crash-work/install/chrono-vsg-r0/lib/cmake/Chrono` (absolute in its cache), `ROBO_DYNA_ENABLE_REPLAY=ON`, and the same dependency prefix/Vulkan environment. Its executable is `crash-work/build/robo-dyna-replay-r0/viewer/robo_dyna_replay`. Exact configure arguments are retained in the guard reports; the viewer cache leaves CUDA, FEA and all mechanics options off.

## Capture and optional capabilities

Chrono's `ExportScreenImage()` delegates to `vsg::write`, and the minimal vsgXchange library unconditionally compiles bundled STB PNG support. GUI font loading uses ImGui's own TTF loader; Chrono also supplies precompiled `.vsgb` label fonts. R0 uses those files, ordinary meshes, no KTX skybox, and no Assimp model import. Preserve Chrono's existing `data/` path. The replay requests 1280x720 with one asset-loading worker and `SetTargetRenderFPS(0)`; it rejects an actual framebuffer above 2560x1440 and verifies every expected PNG after requesting capture before `Render()`.

Assimp v6.0.5, Draco 1.5.7, KTX-Software v4.4.2 and vsgExamples v1.1.13 are additional donor-script pins **not downloaded or audited for licensing here**. They are unnecessary for this geometry/PNG gate; pin their commits and notices only if an admitted asset format needs them. FreeType, CURL, GDAL and OpenEXR are also disabled explicitly so a future host package cannot silently expand the build.

The separate offline encoding step now uses workspace-extracted Ubuntu FFmpeg
`7:4.4.2-0ubuntu0.22.04.1` under `crash-work/install/ffmpeg-r1`, with existing host
shared libraries. Its official package size/SHA match local APT metadata; binary
versions, build configuration and copyright notice are recorded in the
[video provenance](VIDEO.md). No system installation or viewer/solver linkage was
added. The 81-frame coupon video passed metadata checks, full error decode and
visual inspection of decoded first/middle/final frames.

## Loading-worker API and tested lifecycle

The owning `ChVisualSystemVSG.h/.cpp` edits add `SetLoadingThreadCount(int)`: it admits 1 through 64, keeps legacy default 16, rejects invalid counts without mutation, and rejects changes after the first `Initialize()` attempt starts, including failure. A getter exposes configured count. Initialization freezes configuration before plugin callbacks; no existing pool is resized. The 64 cap is an operational API bound, not a limit on driver/internal Vulkan threads. Calls must be externally serialized. The owning VSG unit-test subtree and conditional CMake registration provide the focused tests.

All **three actual CPU tests passed**, in 0 ms of GTest case time, with [retained XML](../../crash-work/reports/chrono-vsg-lifecycle-tests.xml): default 16; valid 1/64/2 before initialization; invalid 0, negative, 65 and `INT_MAX` preserve the prior value; deliberately failing callbacks lock configuration before window/device creation and retain that lock during initialization retry. No Vulkan device was created by these cases. This owning API/test change is committed locally as Chrono `96af26597b`.

The subsequent actual RTX 5090 runtime gates used one worker, verified post-initialize rejection, captured all 15 and 81 expected frames and exited cleanly. The viewer holds each accepted row for two renders because Chrono captures the previous swapchain image, with one initial uncaptured ImGui layout frame. The first normal-rig visual failure remains preserved; existing model-instance wireframe and valid camera-side light APIs corrected presentation without altering the renderer or any geometry. All 96 PNGs passed independent hash/chunk-CRC/dimension checks, every source timestamp matched, and two reviewers inspected first/middle/final geometry and overlays. Both runs stayed below 300 MiB peak sampled RSS and 136 MiB peak device-wide GPU growth, with over 30 GiB VRAM free. See [viewer results](README.md) and the runtime checkpoint for exact limits, outputs and qualification scope. Headless rendering remains unqualified; video has its subsequent separate checkpoint.
