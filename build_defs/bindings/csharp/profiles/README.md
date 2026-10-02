# Configured managed bindings

These thin C# wrappers reuse the same configured native Core, Vehicle, Sensor,
ROS and supporting module DSOs as Python. Core physics and the class factory have
one native implementation owner. The public managed assemblies are
`Robodyna.Managed.Vehicle.OpenCrg.dll`, `Robodyna.Managed.Sensor.dll` and
`Robodyna.Managed.Ros.Sensor.dll`.

Each assembly composes its actual generated proxies once in the retained module
order. The shared composer rejects differing duplicate proxy files unless their
exact hashes and ownership are explicitly reviewed. Select one complete managed
profile assembly per process; independently composed core-proxy assemblies are
not a qualified mixing contract. Existing proxy/PInvoke identities remain stable.

| Profile | Original demonstration targets | Configuration |
| --- | --- | --- |
| OpenCRG | `//examples/csharp/vehicle/opencrg:{road_course,road_course_alternate}` | `--config=opencrg` |
| OptiX Sensor | `//examples/csharp/sensor:camera` | `--config=sensor-optix` |
| ROS Sensor | `//examples/csharp/ros:sensor_bridge` | `--config=sensor-optix --config=ros-sensor` |

Both historical OpenCRG programs actually instantiate VSG, including the source
whose name ends in `_IRR.cs`. All four original source files remain unchanged.
`DemoCatalog.json` records their source hashes and canonical labels.

Host interop and ELF gates are under `//tests/bindings/managed_profiles`, with
matching `opencrg`, `sensor`, and `ros_sensor` test prefixes. They check real road
values and a native step, Sensor/Core shared-body ownership, ROS GPS/TF arguments,
and unique native implementation ownership. Initial profiles exclude additional
FE fields, Multicore, YAML and FSI layouts until separately qualified; the ROS
Sensor profile also excludes the additional URDF handler interface.

The host tests do not construct cameras, lidar or sensor managers, open graphics,
start a ROS bridge, or execute GPU kernels. An OptiX sensor constructor immediately
creates a CUDA stream; camera construction and the original GUI loops require a
separate GPU-guarded runtime. Compilation, host interop, actual graphics and
external ROS delivery are reported separately.
