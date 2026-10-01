"""Native compilation of the absorbed Chrono VSG module, with external SDK only."""

load("@rules_cc//cc:defs.bzl", "cc_library")

VSG_SOURCES = [
    "src/chrono_vsg/ChVisualSystemVSG.cpp",
    "src/chrono_vsg/ChGuiComponentVSG.cpp",
    "src/chrono_vsg/utils/ChDataUtilsVSG.cpp",
    "src/chrono_vsg/utils/ChShaderUtilsVSG.cpp",
    "src/chrono_vsg/utils/ChShapeBuilderVSG.cpp",
    "src/chrono_vsg/impl/BaseGuiComponents.cpp",
    "src/chrono_vsg/impl/BaseEventHandlers.cpp",
    "src/chrono_vsg/impl/VSGnodes.cpp",
]

def chrono_native_vsg(name):
    """Declare VSG in the absorbed Chrono package; shared Xchange profile only.

    Args:
        name: Native rendering module target name.
    """
    cc_library(
        name = name,
        srcs = VSG_SOURCES,
        hdrs = native.glob(["src/chrono_vsg/**/*.h"]),
        includes = ["src"],
        local_defines = ["CH_API_COMPILE_VSG"],
        copts = ["-O3", "-fPIC"],
        deps = [":native_core_fea", ":native_stb", "@vsg_sdk//:sdk"],
        # The SDK's Xchange and ImGui are shared: owning Chrono CMake includes
        # these two STB TUs, while the separate ImGui implementation stays in SDK.
        target_compatible_with = ["@platforms//os:linux"],
        tags = ["manual", "chrono-native-vsg"],
        visibility = ["//visibility:public"],
    )
    native.filegroup(
        name = "vsg_assets",
        srcs = native.glob(["data/vsg/**", "data/colormaps/**", "data/logo_chrono_alpha.png"]),
        visibility = ["//visibility:public"],
    )
