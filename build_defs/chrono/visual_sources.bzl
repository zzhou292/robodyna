"""Explicit ownership for generic objects and visual values; FEA stays enabled."""

VISUAL_SOURCES = {
    "geometry": [
        "src/chrono/geometry/ChAABB.cpp",
        "src/chrono/geometry/ChGeometry.cpp"
    ],
    "material": [
        "src/chrono/assets/ChColor.cpp",
        "src/chrono/assets/ChTexture.cpp",
        "src/chrono/assets/ChVisualMaterial.cpp"
    ],
    "model": [
        "src/chrono/assets/ChCamera.cpp",
        "src/chrono/assets/ChVisualShape.cpp",
        "src/chrono/assets/ChVisualModel.cpp"
    ],
    "object": [
        "src/chrono/physics/ChObject.cpp"
    ]
}

VISUAL_HEADERS = {
    "geometry": [
        "src/chrono/geometry/ChAABB.h",
        "src/chrono/geometry/ChGeometry.h"
    ],
    "material": [
        "src/chrono/assets/ChColor.h",
        "src/chrono/assets/ChTexture.h",
        "src/chrono/assets/ChVisualMaterial.h",
        "src/chrono/assets/ChVisualBSDFType.h"
    ],
    "model": [
        "src/chrono/assets/ChCamera.h",
        "src/chrono/assets/ChVisualShape.h",
        "src/chrono/assets/ChVisualModel.h"
    ],
    "object": [
        "src/chrono/physics/ChObject.h",
        "src/chrono/physics/ChUpdateFlags.h"
    ]
}

VISUAL_TARGETS = {
    "geometry": "//src/geometry:base",
    "material": "//src/visualization/material:values",
    "model": "//src/visualization/model:model",
    "object": "//src/mechanics/object:object"
}

VISUAL_EXTRA_HEADERS = ["src/chrono/core/ChVector2.h"]
VISUAL_ADAPTER_SOURCES = ["//src/fea/visualization:LegacyVisualAdapter.cpp"]
