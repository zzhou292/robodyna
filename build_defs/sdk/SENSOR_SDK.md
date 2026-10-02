# Declared Sensor SDK

`sensor_pins.json` pins 13 Ubuntu 22.04 x86-64 graphics development/runtime packages:
GLFW 3.3.6, GLEW 2.2.0, GLVND 1.4.0 and GLU 9.0.2. Their combined download is
1,363,936 bytes; URLs, versions, sizes, hashes and package dependencies are recorded.
They are extracted under a new workspace prefix, never installed over system
libraries or GPU drivers. Copyright files remain in that prefix.

The separate shader tool uses the already pinned glslang 16.1.0 source archive,
commit `b5782e52ee2f7b3e40bb9c80d15b47016e008bc9`, SHA-256
`6c242598c332050bf3a3619d299594ca2909426935f2c86c69c7356b049347c7`.
The standalone executable links its shader compiler libraries statically; its
ordinary OS/C++ runtime remains a host prerequisite. The qualified VSG shared
libraries and install prefix are unchanged.

The prepared provisioner and guard recipe are in the enclosing workspace at
`crash-work/dependencies/sensor-r0/{prepare_sdk_retry2.py,build-retry2.proposed.json}`. It creates
`crash-work/install/sensor-r1`, retains exact package/source input evidence, builds
with at most four compiler workers, and records generated file hashes and tool
version in `sdk.json`. Run it only through the existing workstation guard.
The first Ninja-generator attempt is retained as failed evidence. The separate
`robodyna-sensor-sdk-build-2.json` guard reports the successful GNU Make retry;
that SDK build result remains distinct from Sensor runtime qualification.

Repository registration:

```starlark
local_sensor_sdk = use_repo_rule("//build_defs/sdk:sensor.bzl", "local_sensor_sdk")
local_sensor_sdk(name = "sensor_sdk")
```

Set `--repo_env=ROBODYNA_SENSOR_ROOT=/absolute/path/to/crash-work/install/sensor-r1`.
The provider verifies the package receipt, shader-source pin, built tool identity,
graphics headers and ELF libraries, and exposes:

- `@sensor_sdk//:windowing`: GL/GLEW/GLFW declarations and native link inputs.
- `@sensor_sdk//:runtime`: the declared shared graphics libraries.
- `@sensor_sdk//:bin/glslangValidator`: the actual build-time GLSL compiler.
- `@sensor_sdk//:sdk.json`: observed dependency evidence.

The Vulkan loader/header pair is reused separately through `@vsg_sdk//:vulkan`.
X11, the host C/C++ ABI, and the installed graphics-driver dispatch remain platform
prerequisites. Neither provider supplies a replacement NVIDIA/Mesa driver or an
inherited Chrono physics binary. Sensor compilation and actual GPU execution are
separate qualification steps.
