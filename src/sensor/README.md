# Native Sensor module: Linux Vulkan RT profile

`//src/sensor:sensor` compiles 54 retained C++ translation units selected by the
original Sensor CMake file. It uses the existing System/core, core-owned tinyobj
loader, and two shared STB units. It does not compile a second mechanics backend,
OptiX kernels, CUDA filtering code, Metal code, or replacement sensor algorithms.

This profile enables the original Vulkan GPU renderer and the original GLFW/GLEW
visualization filters. It compiles five real ray-tracing shaders with glslang
16.1.0 using the original `-V --target-env vulkan1.2` flags. GPU enabling stays
private to the module implementation. The public `CH_API_COMPILE_SENSOR` and
`USE_SENSOR_GLFW` definitions match the retained Sensor CMake declarations.
The unusual public export-definition scope is preserved rather than silently
changed during migration.

`//examples/sensor:native_demos` admits 14 original mains: the retained 11-program
Vulkan CMake set and three reviewed Vehicle scenes already using compatible Sensor
APIs. The complete 16-program catalog keeps HMMWV fog and CRM rendering pending
their actual OptiX implementations. These boundaries are audited in
[OPTIX_PROFILE_AUDIT.md](OPTIX_PROFILE_AUDIT.md).

The SDK is explicit: `@vsg_sdk//:vulkan` reuses the existing Vulkan loader/header
owner, while `@sensor_sdk` supplies the pinned windowing libraries and a separate
standalone GLSL compiler. No qualified VSG library or installed graphics driver
is replaced. See [the SDK recipe](../../build_defs/sdk/SENSOR_SDK.md).

The focused gates check original source bytes and CMake selection, actual compile
ownership, GPU-independent GPS/tachometer behavior, linked windowing versions,
and the five compiled SPIR-V stages. They do not allocate a GPU or open a window.
The inherited renderer contains its original CPU fallback; a successful compile
does not prove that the GPU path was selected. Runtime admission must record the
actual Vulkan device, active renderer, shader inputs, scene output and resource
guard before making a GPU execution claim.

The current generated shader directory is `src/sensor/vulkan/shaders`, relative
to the main repository's runfiles root. The `shaders` target is runtime data of
the module. Ordinary workspace-current-directory execution does not satisfy this
lookup contract. A bounded launcher must resolve that runfiles root and stage the
declared model/output paths before runtime qualification. No absolute build-sandbox
path is embedded and no shader lookup or numerical source was modified.
