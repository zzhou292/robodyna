"""Bind declared package/data roots to the unchanged SolidWorks demo main."""

load("@rules_cc//cc:defs.bzl", "cc_binary")
load("//build_defs/features:defs.bzl", "BASELINE_ABI_ONLY")

def _inputs_impl(ctx):
    ctx.actions.write(ctx.outputs.header, """#pragma once
#include "src/io/parsers/runtime/Resources.h"
namespace {
struct RobodynaDeclaredParserInputs {
    RobodynaDeclaredParserInputs() {
        robodyna::parsers::ConfigureEmbeddedPackage("_main/%s");
        robodyna::parsers::ConfigureDemoData("_main/%s");
    }
};
const RobodynaDeclaredParserInputs robodyna_declared_parser_inputs;
}
""" % (ctx.file.package_manifest.short_path, ctx.file.model.short_path))
    return [DefaultInfo(files = depset([ctx.outputs.header]))]

_inputs = rule(
    implementation = _inputs_impl,
    attrs = {"package_manifest": attr.label(allow_single_file = True), "model": attr.label(allow_single_file = True), "header": attr.output()},
)

def solidworks_demo(name, source):
    model = "//src/compatibility/chrono:data/solidworks/swiss_escapement.py"
    _inputs(name = name + "_inputs", package_manifest = "//bindings/python:core_package_manifest", model = model, header = name + "_inputs.h")
    cc_binary(
        name = name,
        srcs = [source],
        deps = ["//src/io/parsers:python", "//src/io/parsers/runtime:resources", "//src/visualization/irrlicht:irrlicht"],
        dynamic_deps = ["//build_defs/bindings:native_core"],
        data = ["//bindings/python:core_package", "//bindings/python:core_package_manifest", "//src/io/parsers:demo_assets", "//src/compatibility/chrono:irrlicht_assets", "@irrlicht_sdk//:runtime", "@python_embed_sdk//:runtime"],
        copts = ["-include", "$(location :" + name + "_inputs)"],
        additional_compiler_inputs = [":" + name + "_inputs"],
        target_compatible_with = ["@platforms//os:linux", "@platforms//cpu:x86_64"] + BASELINE_ABI_ONLY,
        tags = ["manual", "robodyna-demo", "native-demo", "python-embedding"],
        visibility = ["//visibility:public"],
    )
