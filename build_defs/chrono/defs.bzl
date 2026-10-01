"""Declared, temporary CMake bridge for the absorbed Chrono host implementation.

Native packages will replace this aggregate after dependency extraction. It must
not be used as evidence that FEA and MBD can already link independently.
"""

load("@rules_foreign_cc//foreign_cc:defs.bzl", "cmake")

_OPTIONAL_MODULES = [
    "CASCADE", "CSHARP", "DEM", "FMI", "FSI", "IRRLICHT", "MODAL",
    "MULTICORE", "MUMPS", "PARDISO_MKL", "PARSERS", "PERIDYNAMICS",
    "POSTPROCESS", "PRECICE", "PYTHON", "ROBOT_MODELS", "ROS", "SENSOR",
    "SYNCHRONO", "VEHICLE", "VEHICLE_MODELS", "VSG",
]

def chrono_host_bridge(name, source, compile_jobs = 4):
    """Build core and mechanical FEA from declared first-party sources.

    Args:
        name: Public temporary aggregate target.
        source: Filegroup containing the absorbed Chrono CMake source tree.
        compile_jobs: Nested compiler jobs, bounded independently of Bazel jobs.
    """
    if compile_jobs < 1 or compile_jobs > 4:
        fail("Chrono bridge permits 1 through 4 nested compiler workers")

    cache = {
        "BUILD_BENCHMARKING": "OFF",
        "BUILD_DEMOS": "OFF",
        "BUILD_SHARED_LIBS": "OFF",
        "BUILD_TESTING": "OFF",
        "CH_ENABLE_MODULE_FEA": "ON",
        "CH_ENABLE_MODULE_FEA_MULTIPHYSICS": "OFF",
        "CH_ENABLE_HDF5": "OFF",
        "CH_ENABLE_OPENMP": "OFF",
        "CH_ENABLE_YAML": "OFF",
        "CH_USE_SIMD": "OFF",
        "CHRONO_GPU_VENDOR": "NONE",
        "CMAKE_CXX_STANDARD": "17",
        "CMAKE_POSITION_INDEPENDENT_CODE": "ON",
        "CMAKE_FIND_USE_PACKAGE_REGISTRY": "OFF",
        "CMAKE_FIND_USE_SYSTEM_PACKAGE_REGISTRY": "OFF",
        "CMAKE_DISABLE_FIND_PACKAGE_Thrust": "ON",
        "CMAKE_DISABLE_FIND_PACKAGE_OpenMP": "ON",
        "CMAKE_INSTALL_LIBDIR": "lib",
        # rules_foreign_cc stages the declared @eigen CcInfo include root here.
        # The project hook fails if these headers are absent, before any fallback.
        "EIGEN3_INCLUDE_DIR": "$$EXT_BUILD_DEPS$$/include",
        "CMAKE_PROJECT_Chrono_INCLUDE": "$$EXT_BUILD_ROOT$$/build_defs/chrono/check_inputs.cmake",
        "ROBODYNA_SOURCE_ROOT": "$$EXT_BUILD_ROOT$$",
        "FETCHCONTENT_FULLY_DISCONNECTED": "ON",
        "FETCHCONTENT_UPDATES_DISCONNECTED": "ON",
        "USE_CCACHE": "OFF",
    }
    for module in _OPTIONAL_MODULES:
        cache["CH_ENABLE_MODULE_" + module] = "OFF"

    cmake(
        name = name,
        lib_source = source,
        deps = ["@eigen//:eigen"],
        build_data = ["//build_defs/chrono:check_inputs.cmake",
                      "//include/robodyna/core:RbVector3.h",
                      "//include/robodyna/io:RbPaths.h",
                      "//src/simulation/composition:RbAssembly.cpp",
                      "//include/robodyna/simulation:RbAssembly.h",
                      "//include/robodyna/simulation:RbAssemblyFwd.h",
                      "//src/simulation/composition:RbSystem.cpp",
                      "//include/robodyna/simulation:RbSystem.h",
                      "//src/simulation/composition:RbSystemNSC.cpp",
                      "//include/robodyna/simulation:RbSystemNSC.h",
                      "//src/simulation/composition:RbSystemSMC.cpp",
                      "//include/robodyna/simulation:RbSystemSMC.h",
                      "//include/robodyna/simulation:RbSystemFwd.h",
                      "//src/mechanics/inertia:RbMassProperties.cpp",
                      "//include/robodyna/mechanics:RbMassProperties.h",
                      "//src/mbd/bodies:RbBody.cpp",
                      "//src/mbd/bodies:RbBodyAuxRef.cpp",
                      "//src/mbd/bodies:RbBodyEasy.cpp",
                      "//include/robodyna/mbd:RbBodyAuxRef.h",
                      "//include/robodyna/mbd:RbBodyEasy.h",
                      "//src/fea/mesh:RbMesh.cpp",
                      "//include/robodyna/fea:RbMesh.h",
                      "//include/robodyna/fea:RbMeshFwd.h",
                      "//include/robodyna/mbd:RbBody.h",
                      "//include/robodyna/mbd:RbBodyFwd.h",
                      "//src/fea/visualization:LegacyVisualAdapter.cpp",
                      "//LICENSES:Chrono-BSD-3-Clause.txt"],
        cache_entries = cache,
        generate_args = ["-GNinja"],
        build_args = ["--parallel", str(compile_jobs)],
        targets = ["Chrono_core"],
        out_static_libs = ["libChrono_core.a"],
        defines = ["CH_STATIC"],
        includes = ["chrono_thirdparty", "chrono/collision/bullet", "chrono_thirdparty/HACDv2"],
        linkopts = ["-pthread"],
        # Chrono's archive factory uses registrations in otherwise unreferenced
        # translation units. Keep them until a native explicit registrar exists.
        alwayslink = True,
        target_compatible_with = ["@platforms//os:linux"],
        tags = ["manual", "chrono-transition", "cpu-only"],
    )
