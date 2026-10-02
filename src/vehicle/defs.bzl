"""Compile retained Vehicle implementation groups with one source owner."""

load("@rules_cc//cc:defs.bzl", "cc_library")

def vehicle_components(groups, api_define):
    """Declare compilation groups; they share headers and one composed backend.

    Individual groups are compilation owners, not independently qualified vehicle
    subsystems. The public aggregate preserves the original module symbol closure.
    """
    components = []
    for name, group in groups.items():
        if not group["sources"]:
            continue
        target = name + "_implementation"
        cc_library(
            name = target,
            srcs = ["//src/compatibility/chrono:" + path for path in group["sources"]],
            deps = [":headers"],
            local_defines = [api_define],
            copts = ["-O3", "-fPIC"],
            alwayslink = True,
            target_compatible_with = ["@platforms//os:linux"],
            visibility = ["//visibility:private"],
            tags = ["manual", "native-vehicle", "cpu-dynamics"],
        )
        components.append(":" + target)
    return components
