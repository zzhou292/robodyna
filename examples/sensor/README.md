# Sensor demos

`//examples/sensor:native_demos` groups 14 original Sensor mains under the real
Linux Vulkan RT profile. Individual targets cover GPS/IMU, JSON construction,
cameras, lidar, radar, tachometer, Cornell-box and Vulkan validation scenes.
`DemoCatalog.json` retains all 16 original programs. Three Vehicle scenes extend
the original CMake gate using their existing Vulkan-compatible source. HMMWV fog
and CRM rendering have actual OptiX/SPH targets under `//examples/sensor/optix`.
All 16 original programs passed the current full compile matrix. The catalog's
initial statuses remain historical; compilation does not qualify a GPU scene.

The demo bodies, sensor filtering, update rates, trajectories and output behavior
are unchanged. Their Sensor library, graphics SDK and compiled shaders are native
Bazel dependencies. No old Chrono library or external demo executable is used.

Before runtime, close the selected scene's core/Vehicle/Sensor assets, including
OBJ/MTL textures, JSON-referenced files and HDR environment images. The Vulkan
validation example already checks its selected assets and accepts a data root;
other original mains retain their existing path conventions. Use a fresh output
directory and the workstation guard. The main runfiles root must expose the shader
directory described in [the module contract](../../src/sensor/README.md).

GPU runtime evidence must distinguish an actual Vulkan ray-tracing device from
the inherited fallback. No scene or visualization runtime is qualified by the
initial compile/host-test batch.
