"""Real native example programs using the existing mechanics/module owners.

Feature headers apply only to the example translation units. The qualified core
configuration and its numerical compilation options remain unchanged. Each
advertised optional feature brings its actual implementation and runtime data.
"""

load("@rules_cc//cc:defs.bzl", "cc_binary")

_FEATURES = {
    "sensor": struct(
        macro = "CHRONO_SENSOR",
        deps = ["//src/sensor/backend:sensor"],
        data = ["//src/sensor/backend:runtime_inputs"],
    ),
    "fsi": struct(
        macro = "CHRONO_FSI",
        deps = ["//src/coupling/fsi:fsi"],
        data = [],
    ),
    "fsi_sph": struct(
        macro = "CHRONO_FSI_SPH",
        deps = ["//src/sph:sph"],
        data = [],
    ),
    "mumps": struct(
        macro = "CHRONO_MUMPS",
        deps = ["//src/numerics/sparse:mumps"],
        data = ["@mumps_sdk//:runtime"],
    ),
    "pardiso_mkl": struct(
        macro = "CHRONO_PARDISO_MKL",
        deps = ["//src/numerics/sparse:pardiso_mkl"],
        data = ["@mkl_sdk//:runtime"],
    ),
    "vehicle": struct(
        macro = "CHRONO_VEHICLE",
        deps = ["//src/vehicle:vehicle"],
        data = [],
    ),
    "modal": struct(
        macro = "CHRONO_MODAL",
        deps = ["//src/analysis/modal:modal"],
        data = [],
    ),
    "postprocess": struct(
        macro = "CHRONO_POSTPROCESS",
        deps = ["//src/postprocess:exporters"],
        data = [],
    ),
    "dem": struct(
        macro = "CHRONO_DEM",
        deps = ["//src/dem:dem"],
        data = ["//src/dem:demo_assets"],
    ),
    "irrlicht": struct(
        macro = "CHRONO_IRRLICHT",
        deps = ["//src/visualization/irrlicht:irrlicht"],
        data = ["//src/compatibility/chrono:irrlicht_assets", "@irrlicht_sdk//:runtime"],
    ),
    "vsg": struct(
        macro = "CHRONO_VSG",
        deps = ["//build_defs/chrono/vsg:vsg"],
        data = ["//build_defs/chrono/vsg:assets", "@vsg_sdk//:runtime"],
    ),
}

def _profile_impl(ctx):
    definitions = []
    for feature in ctx.attr.enabled_modules:
        definitions.append("#define " + _FEATURES[feature].macro)
    ctx.actions.write(
        ctx.outputs.header,
        "// Generated demo-only availability profile.\n" +
        "#pragma once\n#include \"chrono/ChConfig.h\"\n" +
        "\n".join(definitions) + "\n",
    )
    return [DefaultInfo(files = depset([ctx.outputs.header]))]

_demo_profile = rule(
    implementation = _profile_impl,
    attrs = {
        "enabled_modules": attr.string_list(),
        "header": attr.output(mandatory = True),
    },
)

def robodyna_cpp_demo(name, srcs, deps = [], data = [], features = [], tags = []):
    """Compile an original main directly with a descriptive Robodyna target name.

    No source/main wrapper or placeholder executable is generated. Additional
    optional modules must acquire a real library owner before entering _FEATURES.
    GUI programs are manual so wildcard test/build operations never launch them.
    """
    if not srcs:
        fail("A Robodyna demo must compile its real source entry point")
    if len(features) != len({feature: True for feature in features}):
        fail("Duplicate demo feature")
    module_deps = []
    module_data = []
    for feature in features:
        if feature not in _FEATURES:
            fail("Demo feature has no native module owner: " + feature)
        module_deps.extend(_FEATURES[feature].deps)
        module_data.extend(_FEATURES[feature].data)
    profile = name + "_availability"
    _demo_profile(name = profile, enabled_modules = features, header = profile + ".h", tags = ["manual"])
    cc_binary(
        name = name,
        srcs = srcs,
        deps = ["//build_defs/chrono:native_core_fea"] + deps + module_deps,
        data = data + module_data,
        copts = ["-include", "$(location :" + profile + ")"],
        additional_compiler_inputs = [":" + profile],
        target_compatible_with = ["@platforms//os:linux"],
        tags = ["manual", "robodyna-demo", "native-demo"] + tags,
        visibility = ["//visibility:public"],
    )
