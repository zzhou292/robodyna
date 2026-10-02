"""Declared original inputs, including the complete Sensor demo denominator."""

load(":sources.bzl", "SENSOR_CONFIG_TEMPLATE", "SENSOR_GROUPS", "SENSOR_PRIVATE_HEADERS", "SENSOR_SHADERS")
load("//examples/sensor:sources.bzl", "SENSOR_DEMO_SOURCES")

SENSOR_SOURCE_EXPORTS = {path: True for path in [
    path
    for group in SENSOR_GROUPS.values()
    for path in group["sources"] + group["headers"]
] + SENSOR_PRIVATE_HEADERS + SENSOR_SHADERS + SENSOR_DEMO_SOURCES + [
    SENSOR_CONFIG_TEMPLATE,
    "src/chrono_sensor/CMakeLists.txt",
    "src/demos/sensor/CMakeLists.txt",
    "src/chrono_thirdparty/stb/stb_image.cpp",
    "src/chrono_thirdparty/stb/stb_image_write.cpp",
    "src/chrono_thirdparty/tinyobjloader/tiny_obj_loader.cc",
]}.keys()
