# Real OptiX Sensor profile

This alternative compiles the original 55 Sensor host translation units and
12 CUDA filter translation units. Enabling the shared FSI/SPH capability adds
the original thirteenth `fsi_sph_render.cu` unit and its actual SPH dependency.
The 12 OptiX shader programs remain NVRTC runtime inputs, with all transitive
project and declared CUDA/CCCL/OptiX/cuRAND headers carried in runfiles.

No Vulkan source, CPU renderer substitute, extra mechanics owner, or rewritten
filter is introduced. The original CUDA filter fast-math policy is local to those
translation units. The production NVRTC path retains its flags and CUDA 12+
OptiX-IR selection. The original function-table definition remains owned once by
`ChOptixEngine.cpp`.

Use `--config=sensor-optix` for the original HMMWV fog scene and
`--config=sensor-optix-sph` for actual SPH/CRM rendering. The latter selects the
coherent shared FSI/SPH and Vehicle CRM headers and implementations. The concrete
Sensor targets reject the opposite backend; `//src/sensor/backend:sensor` selects
one implementation. Baseline language/CMake outputs remain inadmissible under
changed profiles until their own generation and runtime are qualified.

`@optix_sdk` owns pinned OptiX 9.1.0 API headers, NPP 13.1.0.59 and cuRAND
10.4.2.66 headers. It preserves all SDK notices and imports only the three NPP
libraries used by the retained implementation. The CUDA runtime, NVRTC, driver
link metadata and CCCL remain with their existing owners. No NVIDIA driver or
old Chrono binary is installed.

The generated NVRTC include paths come from declared header artifacts, including
their canonical external-repository runfile paths. They are relative to the main
repository runfiles root. Do not run from an arbitrary working directory and
assume the compiler will find ambient SDK headers. The existing shader-directory
setter remains available; no configuration state store is added.

Gates distinguish source/language partition, actual compile ownership, original
CPU scene-parameter behavior, runtime compilation of the original box shader to
OptiX-IR without a device, and eventual guarded GPU execution. Successful NVRTC
compilation does not establish that the NVIDIA driver can create the OptiX
pipeline or that a rendered SPH scene is correct. Those GPU checks remain explicit.
