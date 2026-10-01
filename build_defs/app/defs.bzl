"""Native source-preserving application libraries; no CMake subprocess."""

load("@rules_cc//cc:defs.bzl", "cc_library")

def app_library(name, srcs, hdrs, deps, copts = [], header_deps = []):
    """Keep the existing library boundary while declaring imported source inputs.

    srcs and hdrs are paths below the owned compatibility application directory.
    Compiler options are retained per source target, not globalized.
    """
    cc_library(
        name = name,
        srcs = ["//src/compatibility/app:" + path for path in srcs],
        hdrs = ["//src/compatibility/app:" + path for path in hdrs],
        deps = deps + header_deps + [
            "//src/compatibility/app:include_root",
            "@legacy_fea//lib_src:legacy_include_root",
        ],
        copts = copts,
        visibility = ["//visibility:public"],
    )
