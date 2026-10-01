"""One native compile owner for the image utilities shared by VSG and terrain."""

load("@rules_cc//cc:defs.bzl", "cc_library")

def chrono_native_stb(name):
    """Retain the original two STB implementation files without graphics linkage."""
    cc_library(
        name = name,
        srcs = ["src/chrono_thirdparty/stb/stb_image.cpp", "src/chrono_thirdparty/stb/stb_image_write.cpp"],
        hdrs = native.glob(["src/chrono_thirdparty/stb/*.h"]),
        includes = ["src"],
        copts = ["-O3", "-fPIC"],
        local_defines = ["NDEBUG"],
        target_compatible_with = ["@platforms//os:linux"],
        visibility = ["//visibility:public"],
        tags = ["manual", "cpu-only"],
    )
