"""Original OptiX input exports; runtime shaders remain runtime compiler inputs."""

load(":sources.bzl", "OPTIX_CUDA_SHADER_HEADERS", "OPTIX_CUDA_SOURCES", "OPTIX_HOST_GROUPS", "OPTIX_PRIVATE_HEADERS", "OPTIX_RUNTIME_HEADERS", "OPTIX_RUNTIME_SHADERS", "OPTIX_SPH_CUDA_SOURCES")

OPTIX_SOURCE_EXPORTS = {path: True for path in [
    path
    for group in OPTIX_HOST_GROUPS.values()
    for path in group["sources"] + group["headers"]
] + OPTIX_CUDA_SHADER_HEADERS + OPTIX_CUDA_SOURCES + OPTIX_SPH_CUDA_SOURCES + OPTIX_PRIVATE_HEADERS + OPTIX_RUNTIME_HEADERS + OPTIX_RUNTIME_SHADERS + [
    "src/chrono_sensor/ChConfigSensor.h.in",
    "src/chrono_sensor/CMakeLists.txt",
    "src/demos/sensor/demo_SEN_HMMWV.cpp",
    "src/demos/sensor/demo_SEN_CRM_Rendering.cpp",
    "src/demos/robot/curiosity/demo_ROBOT_Curiosity_SCM_Sensor.cpp",
    "src/demos/robot/viper/demo_ROBOT_Viper_SCM_Sensor.cpp",
]}.keys()
