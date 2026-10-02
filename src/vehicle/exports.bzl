"""Exact compatibility-source exports consumed by the owned Vehicle libraries."""

load(":sources.bzl", "VEHICLE_GROUPS")
load("//src/vehicle/models:sources.bzl", "MODEL_GROUPS")
load("//src/vehicle/visualization:sources.bzl", "VISUALIZATION_GROUPS")
load("//examples/vehicle:sources.bzl", "ARTICULATED_HEADERS", "ARTICULATED_SOURCES", "VEHICLE_DEMO_CMAKE_FILES", "VEHICLE_DEMO_HEADERS", "VEHICLE_DEMO_SOURCES")

def _unique(values):
    return {value: True for value in values}.keys()

VEHICLE_SOURCE_EXPORTS = _unique([
    path
    for groups in [VEHICLE_GROUPS, MODEL_GROUPS, VISUALIZATION_GROUPS]
    for group in groups.values()
    for path in group["sources"] + group["headers"]
] + [
    "src/chrono_vehicle/CMakeLists.txt",
    "src/chrono_models/vehicle/CMakeLists.txt",
    "src/chrono_vehicle/ChConfigVehicle.h.in",
    "src/chrono_vehicle/ChTerrain.cpp",
    "src/chrono_vehicle/ChWorldFrame.cpp",
    "src/chrono_vehicle/ChVehicleDataPath.cpp",
    "src/chrono_vehicle/terrain/SCMTerrain.cpp",
    "src/chrono_vehicle/visualization/ChScmVisualizationVSG.cpp",
    "src/chrono_thirdparty/stb/stb_image.cpp",
    "src/chrono_thirdparty/stb/stb_image_write.cpp",
])

VEHICLE_DEMO_EXPORTS = _unique(
    VEHICLE_DEMO_SOURCES + VEHICLE_DEMO_HEADERS + VEHICLE_DEMO_CMAKE_FILES +
    ARTICULATED_SOURCES + ARTICULATED_HEADERS,
)
