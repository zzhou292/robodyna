"""Declared retained inputs for robot models and their complete demo inventory."""

load(":sources.bzl", "ROBOT_MODEL_GROUPS", "ROBOT_PENDING_URDF")
load("//examples/robotics:sources.bzl", "ROBOT_DEMO_CMAKE_FILES", "ROBOT_DEMO_HEADERS", "ROBOT_DEMO_HELPERS", "ROBOT_DEMO_SOURCES")

ROBOT_SOURCE_EXPORTS = {path: True for path in [
    path
    for group in ROBOT_MODEL_GROUPS.values()
    for path in group["sources"] + group["headers"]
] + ROBOT_PENDING_URDF + ROBOT_DEMO_SOURCES + ROBOT_DEMO_HEADERS + ROBOT_DEMO_HELPERS + ROBOT_DEMO_CMAKE_FILES + [
    "src/chrono_models/robot/CMakeLists.txt",
    "src/chrono_vehicle/ChConfigVehicle.h.in",
    "src/chrono_vehicle/wheeled_vehicle/test_rig/ChWheelTestRig.h",
    "src/chrono_vehicle/wheeled_vehicle/test_rig/ChWheelTestRig.cpp",
]}.keys()
