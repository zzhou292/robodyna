"""Real native ownership for unchanged, domain-neutral Chrono implementation."""

load("@rules_cc//cc:defs.bzl", "cc_library")
load(":neutral_sources.bzl", "NEUTRAL_HEADERS", "NEUTRAL_SOURCES", "NEUTRAL_SOURCE_RELOCATIONS", "NEUTRAL_HEADER_TARGETS")

_SOURCE_ROOT = "//src/compatibility/chrono:"

def neutral_library(name, component, deps):
    """Compile one neutral component without an aggregate header dependency.

    Args:
        name: Real public library target, not an alias to the inherited aggregate.
        component: Key of the reviewed source/header ownership manifest.
        deps: Narrow foundation or variable-block dependencies.
    """
    cc_library(
        name = name,
        srcs = [NEUTRAL_SOURCE_RELOCATIONS.get(path, _SOURCE_ROOT + path) for path in NEUTRAL_SOURCES[component]],
        hdrs = [_SOURCE_ROOT + path for path in NEUTRAL_HEADERS[component]],
        deps = deps + NEUTRAL_HEADER_TARGETS.get(component, []) + [
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
        additional_compiler_inputs = [_SOURCE_ROOT + "src/chrono/ChCorePCH.h"],
        alwayslink = True,
        target_compatible_with = ["@platforms//os:linux"],
        visibility = ["//visibility:public"],
        tags = ["manual", "neutral-mechanics", "cpu-only"],
    )
