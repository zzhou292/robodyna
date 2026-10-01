"""Compile an explicitly owned native component using the qualified host flags."""

load("@rules_cc//cc:defs.bzl", "cc_library")

def native_component_library(name, srcs, hdrs, deps, tags):
    """Share compile policy without sharing unrelated domain sources or headers."""
    cc_library(
        name = name,
        srcs = srcs,
        hdrs = hdrs,
        deps = deps + [
            "//src/core/configuration:host_headers",
            "//src/compatibility/chrono:neutral_include_root",
            "@eigen//:eigen",
        ],
        defines = ["CH_STATIC", "CH_IGNORE_DEPRECATED", "EIGEN_DONT_PARALLELIZE", "_ENABLE_EXTENDED_ALIGNED_STORAGE", "NDEBUG"],
        local_defines = ["CH_API_COMPILE", "BT_THREADSAFE", "BP_USE_FIXEDPOINT_INT_32"],
        copts = [
            "-O3", "-fPIC", "-include", "$(location //src/compatibility/chrono:src/chrono/ChCorePCH.h)",
            "-Wint-in-bool-context", "-Wno-sign-compare", "-Wno-reorder", "-Wno-unused-function",
            "-Wno-unused-parameter", "-Wno-unused-result", "-Wno-deprecated",
        ],
        additional_compiler_inputs = ["//src/compatibility/chrono:src/chrono/ChCorePCH.h"],
        alwayslink = True,
        target_compatible_with = ["@platforms//os:linux"],
        visibility = ["//visibility:public"],
        tags = tags,
    )
