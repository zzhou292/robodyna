"""Verify real model/helper compile owners without importing a second backend."""

load("@rules_cc//cc/common:cc_info.bzl", "CcInfo")
load("//build_defs/chrono:dependency_closure.bzl", "NativeClosure", "native_closure_aspect")
load(":sources.bzl", "ROBOT_MODEL_GROUPS")

def _ownership_impl(ctx):
    prefix = "src/compatibility/chrono/"
    expected = {prefix + path: str(Label("//src/robotics/models:models")) for group in ROBOT_MODEL_GROUPS.values() for path in group["sources"]}
    expected[prefix + "src/demos/robot/lander/model/Lander.cpp"] = str(Label("//examples/robotics:lander_model"))
    found = {}
    for target in [ctx.attr.models, ctx.attr.lander]:
        for entry in target[NativeClosure].owners.to_list():
            source, owner = entry.split("|", 1)
            if source.startswith(prefix + "src/chrono_models/robot/") or source.startswith(prefix + "src/demos/robot/"):
                if source not in expected or expected[source] != owner:
                    fail("Unexpected robot model/helper compile owner: " + entry)
                found[source] = owner
    if found != expected:
        fail("Robot model/helper closure omits a retained translation unit")
    for entry in ctx.attr.models[NativeClosure].owners.to_list():
        source, _ = entry.split("|", 1)
        if source.startswith(prefix + "src/chrono_vehicle/") or source.startswith(prefix + "src/chrono_models/vehicle/") or source.startswith(prefix + "src/chrono_fsi/"):
            fail("Robot model library acquired a Vehicle or FSI implementation dependency: " + entry)
    if "CH_API_COMPILE_MODELS" in ctx.attr.models[CcInfo].compilation_context.defines.to_list():
        fail("Robot implementation export definition leaked to consumers")
    executable = ctx.actions.declare_file(ctx.label.name + ".sh")
    ctx.actions.write(executable, "#!/bin/sh\n# Actual robot compile ownership was verified during Bazel analysis.\nexit 0\n", is_executable = True)
    return [DefaultInfo(executable = executable)]

robotics_ownership_test = rule(
    implementation = _ownership_impl,
    attrs = {
        "models": attr.label(mandatory = True, aspects = [native_closure_aspect], providers = [CcInfo]),
        "lander": attr.label(mandatory = True, aspects = [native_closure_aspect]),
    },
    test = True,
)
