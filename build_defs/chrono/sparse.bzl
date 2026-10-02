"""Compile retained sparse interfaces against actual admitted vendor SDKs."""

load("@rules_cc//cc:defs.bzl", "cc_library")

def chrono_native_mumps(name):
    cc_library(
        name = name,
        srcs = ["src/chrono_mumps/ChMumpsEngine.cpp", "src/chrono_mumps/ChSolverMumps.cpp"],
        hdrs = ["src/chrono_mumps/ChApiMumps.h", "src/chrono_mumps/ChMumpsEngine.h", "src/chrono_mumps/ChSolverMumps.h"],
        includes = ["src"],
        deps = [":native_core_fea", "@mumps_sdk//:sdk"],
        local_defines = ["CH_API_COMPILE_MUMPS", "CH_IGNORE_DEPRECATED", "_OPENMP_NOFORCE_MANIFEST"],
        copts = ["-O3", "-fPIC"],
        target_compatible_with = ["@platforms//os:linux"],
        tags = ["manual", "native-mumps", "cpu-only"],
        visibility = ["//visibility:public"],
    )

def chrono_native_pardisomkl(name):
    cc_library(
        name = name,
        srcs = ["src/chrono_pardisomkl/ChSolverPardisoMKL.cpp"],
        hdrs = ["src/chrono_pardisomkl/ChApiPardisoMKL.h", "src/chrono_pardisomkl/ChSolverPardisoMKL.h"],
        includes = ["src"],
        deps = [":native_core_fea", "@mkl_sdk//:sdk"],
        # Retain the owning CMake's PRIVATE numerical-interface definitions.
        # They do not propagate EIGEN_USE_MKL_ALL into the qualified core.
        local_defines = ["CH_API_COMPILE_PARDISOMKL", "EIGEN_USE_MKL_ALL", "MKL_LP64"],
        copts = ["-O3", "-fPIC"],
        target_compatible_with = ["@platforms//os:linux"],
        tags = ["manual", "native-pardiso-mkl", "cpu-only"],
        visibility = ["//visibility:public"],
    )
