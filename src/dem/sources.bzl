"""Explicit retained DEM source groups, matching the owning CMake module.

Paths are relative to src/compatibility/chrono. Header exports do not select
additional translation units. The backend retains its existing CUDA equations.
"""

DEM_HOST_SOURCES = [
    "src/chrono_dem/physics/ChSystemDem.cpp",
    "src/chrono_dem/physics/ChSystemDem_impl.cpp",
    "src/chrono_dem/physics/ChSystemDemMesh_impl.cpp",
]

DEM_CUDA_SOURCES = [
    "src/chrono_dem/gpu/ChDemSMC.cu",
    "src/chrono_dem/gpu/ChDemSMCtrimesh.cu",
]

DEM_HEADERS = [
    "src/chrono_dem/ChApiDem.h",
    "src/chrono_dem/ChDemDefines.h",
    "src/chrono_dem/physics/ChSystemDem.h",
    "src/chrono_dem/physics/ChSystemDem_impl.h",
    "src/chrono_dem/physics/ChSystemDemMesh_impl.h",
    "src/chrono_dem/physics/ChDemBoundaryConditions.h",
    "src/chrono_dem/gpu/ChDemSMC.cuh",
    "src/chrono_dem/gpu/ChDemSMCtrimesh.cuh",
    "src/chrono_dem/gpu/ChDemCollision.cuh",
    "src/chrono_dem/gpu/ChDemBoundaryConditions.cuh",
    "src/chrono_dem/gpu/ChDemHelpers.cuh",
    "src/chrono_dem/gpu/ChDemBoxTriangle.cuh",
    "src/chrono_dem/gpu/ChDemGpuAlloc.h",
    "src/chrono_dem/gpu/ChDemGpuMathUtils.cuh",
    "src/chrono_dem/utils/ChDemUtilities.h",
    "src/chrono_dem/utils/ChDemJsonParser.h",
    "src/chrono_dem/utils/ChDemSphereDecomp.h",
]

DEM_VISUAL_SOURCES = ["src/chrono_dem/visualization/ChDemVisualizationVSG.cpp"]
DEM_VISUAL_HEADERS = ["src/chrono_dem/visualization/ChDemVisualizationVSG.h"]
DEM_CONFIG_TEMPLATE = "src/chrono_dem/ChConfigDem.h.in"
DEM_CMAKE = "src/chrono_dem/CMakeLists.txt"
DEM_DEMO_CMAKE = "src/demos/dem/CMakeLists.txt"

DEM_DEMO_SOURCES = [
    "src/demos/dem/demo_DEM_ballCosim.cpp",
    "src/demos/dem/demo_DEM_movingBoundary.cpp",
    "src/demos/dem/demo_DEM_fixedTerrain.cpp",
    "src/demos/dem/demo_DEM_mixer.cpp",
    "src/demos/dem/demo_DEM_repose.cpp",
]

DEM_DATA_FILES = [
    "data/dem/ballCosim.json",
    "data/dem/movingBoundary.json",
    "data/dem/fixedTerrain.json",
    "data/dem/mixer.json",
    "data/dem/repose.json",
    "data/models/sphere.obj",
    "data/models/fixedterrain.obj",
    "data/models/mixer/internal_mixer.obj",
    "data/models/funnel.obj",
]

DEM_SOURCE_EXPORTS = (
    DEM_HOST_SOURCES + DEM_CUDA_SOURCES + DEM_HEADERS +
    DEM_VISUAL_SOURCES + DEM_VISUAL_HEADERS + DEM_DEMO_SOURCES +
    [DEM_CONFIG_TEMPLATE, DEM_CMAKE, DEM_DEMO_CMAKE]
)
