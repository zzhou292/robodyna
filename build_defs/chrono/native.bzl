"""Native Bazel build of the initial inherited CPU mechanics aggregate.

Source lists are explicit and source-derived. This target changes build ownership,
not numerical algorithms, state ownership or the FEA/MBD coupling method.
"""

load("@rules_cc//cc:defs.bzl", "cc_library")
load(":native_sources.bzl", "NATIVE_SOURCE_GROUPS")
load(":source_paths.bzl", "current_source_label")
load(":neutral_sources.bzl", "NEUTRAL_SOURCES", "NEUTRAL_TARGETS")
load(":visual_sources.bzl", "VISUAL_ADAPTER_SOURCES", "VISUAL_SOURCES", "VISUAL_TARGETS")

_VENDOR_GROUPS = [
    "collision_bullet",
    "collision_convexdecompHACDv2",
    "collision_convexdecompVHACD",
    "tiny_obj",
    "libstl",
]
_VENDOR_HEADERS = [
    "HACDv2", "VHACD", "tinyobjloader", "libstl", "filesystem", "cxxopts", "rapidjson", "rapidxml",
]
_TAGS = ["manual", "chrono-native-aggregate", "cpu-only"]

def chrono_native_host(name):
    """Declare an aggregate in the absorbed source-root package only.

    Args:
        name: Native aggregate target name; it must not claim domain independence.
    """
    native.alias(
        name = name + "_configuration_headers",
        actual = "//src/core/configuration:host_headers",
        tags = _TAGS,
    )

    # Header export matches the inherited include roots. Header globs do not
    # decide which algorithms compile: implementation sources are explicit below.
    header_patterns = ["src/chrono/**/*.h", "src/chrono/**/*.inl"]
    for vendor in _VENDOR_HEADERS:
        for extension in ["h", "hpp", "inl", "inc"]:
            header_patterns.append("src/chrono_thirdparty/" + vendor + "/**/*." + extension)
    # Not every retained directory uses every header extension. Bazel 9 rejects
    # an individual empty pattern unless explicitly permitted. Keep a separate
    # required-header check so a missing source import still fails clearly.
    headers = native.glob(header_patterns, allow_empty = True)
    for required in ["src/chrono/physics/ChBody.h", "src/chrono/fea/ChMesh.h"]:
        if required not in headers:
            fail("Required absorbed Chrono header is missing: " + required)
    cc_library(
        name = name + "_headers",
        hdrs = headers,
        includes = ["src", "src/chrono/collision/bullet", "src/chrono_thirdparty", "src/chrono_thirdparty/HACDv2"],
        defines = ["CH_STATIC", "CH_IGNORE_DEPRECATED", "EIGEN_DONT_PARALLELIZE", "_ENABLE_EXTENDED_ALIGNED_STORAGE", "NDEBUG"],
        deps = [":" + name + "_configuration_headers", "@eigen//:eigen",
                "//include/robodyna/mechanics:inertia_headers", "//include/robodyna/mbd:implementation_headers", "//include/robodyna/fea:implementation_headers"],
        tags = _TAGS,
    )
    common = {
        "alwayslink": True,
        "deps": [":" + name + "_headers"],
        "local_defines": ["CH_API_COMPILE", "BT_THREADSAFE", "BP_USE_FIXEDPOINT_INT_32"],
        "target_compatible_with": ["@platforms//os:linux"],
        "tags": _TAGS,
    }
    core_sources = []
    vendor_sources = []
    extracted = {path: True for paths in NEUTRAL_SOURCES.values() + VISUAL_SOURCES.values() for path in paths}
    for group, paths in NATIVE_SOURCE_GROUPS.items():
        if group in _VENDOR_GROUPS:
            vendor_sources.extend(paths)
        else:
            core_sources.extend([current_source_label(path) for path in paths if path not in extracted])
    cc_library(
        name = name + "_implementation",
        srcs = core_sources + VISUAL_ADAPTER_SOURCES,
        # The inherited PCH installs Eigen plugins before any other headers.
        # Preserve that include order without inventing a Bazel PCH toolchain.
        copts = [
            "-O3", "-fPIC", "-include", "$(location src/chrono/ChCorePCH.h)",
            "-Wint-in-bool-context", "-Wno-sign-compare", "-Wno-reorder", "-Wno-unused-function",
            "-Wno-unused-parameter", "-Wno-unused-result", "-Wno-deprecated",
        ],
        # Compiler-option location expansion uses additional_compiler_inputs;
        # a transitive/textual header alone does not enter that prerequisite map.
        additional_compiler_inputs = ["src/chrono/ChCorePCH.h"],
        **common
    )
    cc_library(
        name = name + "_bundled_collision",
        srcs = vendor_sources,
        # Chrono's owning CMake also disables PCH/warnings for these sources.
        copts = ["-O3", "-fPIC", "-w"],
        **common
    )
    cc_library(
        name = name,
        deps = [":" + name + "_implementation", ":" + name + "_bundled_collision"] + NEUTRAL_TARGETS.values() + VISUAL_TARGETS.values(),
        linkopts = ["-pthread"],
        target_compatible_with = ["@platforms//os:linux"],
        tags = _TAGS,
    )
