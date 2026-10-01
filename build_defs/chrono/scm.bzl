"""Native retained CPU SCM terrain and its optional existing VSG plugin.

Only the precise source closure needed by the original rigid-tire example is
selected. No soil, ray-casting, bulldozing or contact algorithm is rewritten.
"""

load("@rules_cc//cc:defs.bzl", "cc_library")

SCM_SOURCES = [
    "src/chrono_vehicle/ChTerrain.cpp",
    "src/chrono_vehicle/ChWorldFrame.cpp",
    "src/chrono_vehicle/ChVehicleDataPath.cpp",
    "src/chrono_vehicle/terrain/SCMTerrain.cpp",
]

def _scm_configuration_impl(ctx):
    ctx.actions.expand_template(
        template = ctx.file.template,
        output = ctx.outputs.header,
        substitutions = {
            "@CHRONO_OPENCRG@": "#undef CHRONO_OPENCRG",
            "@CHRONO_CRM@": "#undef CHRONO_CRM",
            "@CHRONO_SCM_GPU@": "#undef CHRONO_HAS_SCM_GPU",
        },
    )
    return [DefaultInfo(files = depset([ctx.outputs.header]))]

_scm_configuration = rule(
    implementation = _scm_configuration_impl,
    attrs = {
        "template": attr.label(allow_single_file = True, mandatory = True),
        "header": attr.output(mandatory = True),
    },
)

def chrono_native_scm(name):
    """Declare the CPU/Bullet retention profile and separate presentation target."""
    _scm_configuration(
        name = name + "_configuration",
        template = "src/chrono_vehicle/ChConfigVehicle.h.in",
        header = name + "_config/include/chrono_vehicle/ChConfigVehicle.h",
    )
    cc_library(
        name = name + "_configuration_headers",
        hdrs = [":" + name + "_configuration"],
        strip_include_prefix = name + "_config/include",
        deps = [":native_core_fea_configuration_headers"],
    )
    cc_library(
        name = name,
        srcs = SCM_SOURCES,
        hdrs = [
            "src/chrono_vehicle/ChApiVehicle.h",
            "src/chrono_vehicle/ChTerrain.h",
            "src/chrono_vehicle/ChWorldFrame.h",
            "src/chrono_vehicle/ChVehicleDataPath.h",
            "src/chrono_vehicle/ChSubsysDefs.h",
            "src/chrono_vehicle/terrain/SCMTerrain.h",
        ],
        includes = ["src"],
        deps = [":native_core_fea", ":native_stb", ":" + name + "_configuration_headers"],
        local_defines = ["CH_API_COMPILE_VEHICLE"],
        copts = ["-O3", "-fPIC"],
        target_compatible_with = ["@platforms//os:linux"],
        visibility = ["//visibility:public"],
        tags = ["manual", "cpu-only", "retained-scm"],
    )
    cc_library(
        name = name + "_vsg",
        srcs = ["src/chrono_vehicle/visualization/ChScmVisualizationVSG.cpp"],
        hdrs = ["src/chrono_vehicle/visualization/ChScmVisualizationVSG.h"],
        includes = ["src"],
        deps = [":" + name, ":native_vsg"],
        local_defines = ["CH_API_COMPILE_VEHICLE"],
        copts = ["-O3", "-fPIC"],
        target_compatible_with = ["@platforms//os:linux"],
        visibility = ["//visibility:public"],
        tags = ["manual", "retained-scm"],
    )
