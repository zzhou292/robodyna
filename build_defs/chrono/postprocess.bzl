"""Native ownership of the retained scene exporters; no external solver or core copy."""

load("@rules_cc//cc:defs.bzl", "cc_library")

POSTPROCESS_SOURCES = [
    "src/chrono_postprocess/ChPovRay.cpp",
    "src/chrono_postprocess/ChBlender.cpp",
]

POSTPROCESS_HEADERS = [
    "src/chrono_postprocess/ChApiPostProcess.h",
    "src/chrono_postprocess/ChPostProcessBase.h",
    "src/chrono_postprocess/ChPovRay.h",
    "src/chrono_postprocess/ChBlender.h",
    "src/chrono_postprocess/ChGnuPlot.h",
]

def chrono_native_postprocess(name):
    """Compile the two inherited CMake units once, against the existing core.

    Gnuplot support is header-only. This target does not claim that its external
    plotting executable is available or enable CHRONO_HAS_GNUPLOT implicitly.
    """
    cc_library(
        name = name,
        srcs = POSTPROCESS_SOURCES,
        hdrs = POSTPROCESS_HEADERS,
        includes = ["src"],
        local_defines = ["CH_API_COMPILE_POSTPROCESS"],
        copts = ["-O3", "-fPIC"],
        deps = [":native_core_fea"],
        target_compatible_with = ["@platforms//os:linux"],
        visibility = ["//visibility:public"],
        tags = ["manual", "native-postprocess", "cpu-only"],
    )
