"""Explicit retained peridynamics source ownership from its CMake target."""

PERIDYNAMICS_SOURCES = [
    "src/chrono_peridynamics/ChNodePeri.cpp",
    "src/chrono_peridynamics/ChMatterPeriSprings.cpp",
    "src/chrono_peridynamics/ChMatterPeriBB.cpp",
    "src/chrono_peridynamics/ChMatterPeriBBimplicit.cpp",
    "src/chrono_peridynamics/ChMatterPeriLinearElastic.cpp",
    "src/chrono_peridynamics/ChMatterPeriLiquid.cpp",
    "src/chrono_peridynamics/ChPeridynamics.cpp"
]

PERIDYNAMICS_HEADERS = [
    "src/chrono_peridynamics/ChApiPeridynamics.h",
    "src/chrono_peridynamics/ChNodePeri.h",
    "src/chrono_peridynamics/ChMatterPeridynamics.h",
    "src/chrono_peridynamics/ChMatterPeriSprings.h",
    "src/chrono_peridynamics/ChMatterPeriBB.h",
    "src/chrono_peridynamics/ChMatterPeriBBimplicit.h",
    "src/chrono_peridynamics/ChMatterPeriLinearElastic.h",
    "src/chrono_peridynamics/ChMatterPeriLiquid.h",
    "src/chrono_peridynamics/ChPeridynamics.h"
]

PERIDYNAMICS_EXPORTS = PERIDYNAMICS_SOURCES + PERIDYNAMICS_HEADERS + ["src/chrono_peridynamics/CMakeLists.txt"]
