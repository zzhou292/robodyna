# Remaining Sensor demo profiles

The initial native target preserves the retained Linux Vulkan configuration.
The complete 16-demo denominator includes these five cases outside its original
CMake Vulkan gate. Three have now been admitted for native compilation only:

| Original main | Present admission boundary |
| --- | --- |
| `demo_SEN_Gator` | Admitted for compilation: existing camera/lidar/IMU/GPS/filter interfaces support the selected Vulkan backend; native Vehicle models and Irrlicht are declared |
| `demo_SEN_HMMWV` | Pending real OptiX fog. Its line 291 calls `SetFogScatteringFromDistance(200)`; neither the Vulkan scene nor its shaders implement that behavior. The native compile gate exposed this gap |
| `demo_SEN_deformableSoil` | Admitted for compilation using existing camera and scene interfaces plus actual Vehicle SCM and Irrlicht owners |
| `demo_SEN_metal_quarterpanel` | Admitted for compilation: the original main explicitly selects Vulkan at lines 54–63; native Vehicle JSON/model closure is declared |
| `demo_SEN_CRM_Rendering` | Requires both coherent Vehicle CRM/SPH and actual Sensor SPH rendering. The main calls `AttachFsiSphSystem` at line 295; `ChSensorManager.cpp:275` only attaches under OptiX and returns `-1` otherwise |

The three gate extensions change no source, scene setup or backend algorithm.
Their actual compilation and runtime receipts remain separate from admission.
Compiling the CRM scene with the Vulkan profile could produce a binary
that omits the requested fluid rendering. That is not coverage of this example.
No such substitution is included in the initial target.

NVIDIA's public [optix-dev repository](https://github.com/NVIDIA/optix-dev)
provides the minimal headers and driver API access for building OptiX applications.
It is a viable source for a separate real backend profile; the full sample SDK is
not required merely to obtain the API headers. Its
[license information](https://github.com/NVIDIA/optix-dev/blob/main/license_info.txt)
distinguishes BSD and NVIDIA proprietary headers in addition to the SDK terms.
Retain those notices in the external dependency package; do not label the complete
SDK as Robodyna's BSD code.

The concrete next profile needs:

1. A pinned official OptiX header release and the installed NVIDIA driver API.
   The retained `ChOptixUtils.cpp` calls `optixModuleCreate`, requiring the 7.7-or-newer
   API; a specific modern version still needs compilation qualification.
2. The original 12 CUDA filter units, seven OptiX C++ units, eight OptiX sensor
   classes, additional filters, and all shader/header inputs selected by CMake.
   Reuse one selected CUDA runtime, plus actual NPP `nppc/nppig/nppidei`, cuRAND
   headers, NVRTC and CCCL dependencies. The current Vulkan profile needs none of
   those CUDA filtering libraries.
3. Preserve the existing NVRTC option/flag and shader-include list. On CUDA 12+
   `ChOptixUtils.cpp:35–48` and its compilation path request OptiX-IR. This retained
   path is intentional for Blackwell; do not switch it back to ordinary PTX while
   wiring the build. Preserve original CUDA arithmetic flags for this profile.
4. An explicit resource directory for runtime shader compilation, with all
   transitive include files, plus a runtime driver/backend gate. Header availability
   is not evidence that those shaders load or the driver accepts the pipeline.
5. For the CRM scene, a coherent SPH-enabled Sensor profile and Vehicle CRM profile
   across headers, implementations and consumers. Optional NanoVDB remains off
   unless a scene and its separate dependency closure require it.

OptiX and Vulkan profiles must not coexist as duplicate Sensor class/implementation
owners in one binary. A mixed backend configuration, if requested, must follow the
retained CMake composition and receive its own common-header and runtime checks.
