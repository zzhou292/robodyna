"""Single compile owners for the retained MPI co-simulation components."""

load("@rules_cc//cc:defs.bzl", "cc_library")

def cosim_component(name, group, deps, compatible = []):
    """Compile one original family, sharing configuration with all consumers."""
    cc_library(
        name = name,
        srcs = ["//src/compatibility/chrono:" + path for path in group["sources"]],
        hdrs = ["//src/compatibility/chrono:" + path for path in group["headers"]],
        strip_include_prefix = "/src/compatibility/chrono/src",
        deps = deps,
        local_defines = ["CH_API_COMPILE_VEHICLE"],
        copts = ["-O3", "-fPIC"],
        alwayslink = True,
        target_compatible_with = ["@platforms//os:linux"] + compatible,
        tags = ["manual", "native-vehicle-cosim"],
    )
