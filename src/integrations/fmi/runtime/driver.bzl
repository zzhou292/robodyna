"""Supply declared artifact paths to the unchanged retained FMI entrypoints."""

load("@rules_cc//cc:defs.bzl", "cc_binary")
load("//build_defs/features:defs.bzl", "BASELINE_ABI_ONLY")
load("//src/integrations/fmi/export:defs.bzl", "FmuInfo")

def _configuration_impl(ctx):
    lines = [
        "#pragma once",
        '#include "chrono/ChConfig.h"',
        '#include "src/integrations/fmi/runtime/Resources.h"',
        '#define DEMO_FMU_MAIN_DIR "."',
        '#define FMU_UNPACK_DIRECTORY "./tmp_unpack_template"',
        '#define FMU_DIRECTORY "."',
    ]
    if ctx.attr.visualization:
        lines.append("#define CHRONO_VSG")
    for target, macro in ctx.attr.fmu_macros.items():
        file = target[FmuInfo].archive
        lines.append('#define %s robodyna::fmi::ArchiveRunfile("_main/%s")' % (macro, file.short_path))
    ctx.actions.write(ctx.outputs.header, "\n".join(lines) + "\n")
    return [DefaultInfo(files = depset([ctx.outputs.header]))]

_driver_configuration = rule(
    implementation = _configuration_impl,
    attrs = {
        "fmu_macros": attr.label_keyed_string_dict(providers = [FmuInfo]),
        "visualization": attr.bool(default = False),
        "header": attr.output(mandatory = True),
    },
)

def native_fmi_driver(name, source, fmu_macros, external_wrapper = False, visualization = False):
    profile = name + "_inputs"
    _driver_configuration(
        name = profile, fmu_macros = fmu_macros, visualization = visualization,
        header = profile + ".h", tags = ["manual"],
    )
    deps = ["//src/integrations/fmi:headers", "//src/integrations/fmi/runtime:resources"]
    data = list(fmu_macros.keys())
    if external_wrapper:
        deps.append("//src/integrations/fmi:fmi")
    if visualization:
        deps.append("//build_defs/chrono/vsg:vsg")
        data.extend(["//build_defs/chrono/vsg:assets", "@vsg_sdk//:runtime"])
    cc_binary(
        name = name,
        srcs = [source],
        deps = deps,
        data = data,
        copts = ["-include", "$(location :" + profile + ")"],
        additional_compiler_inputs = [":" + profile],
        target_compatible_with = ["@platforms//os:linux", "@platforms//cpu:x86_64"] + BASELINE_ABI_ONLY,
        tags = ["manual", "robodyna-demo", "native-demo", "native-fmi", "cpu-dynamics"],
        visibility = ["//visibility:public"],
    )
