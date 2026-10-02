"""Reject mixed Sensor owners and verify the profile's actual source partition."""

load("@rules_cc//cc/common:cc_info.bzl", "CcInfo")
load("//build_defs/chrono:dependency_closure.bzl", "NativeClosure", "native_closure_aspect")
load(":sources.bzl", "OPTIX_CUDA_SOURCES", "OPTIX_HOST_GROUPS", "OPTIX_SPH_CUDA_SOURCES")

def _ownership_impl(ctx):
    prefix = "src/compatibility/chrono/"
    expected = {prefix + path: str(Label("//src/sensor/optix:sensor")) for group in OPTIX_HOST_GROUPS.values() for path in group["sources"]}
    for path in OPTIX_CUDA_SOURCES + (OPTIX_SPH_CUDA_SOURCES if ctx.attr.with_sph else []):
        expected[prefix + path] = str(Label("//src/sensor/optix:kernels"))
    found = {}
    for entry in ctx.attr.implementation[NativeClosure].owners.to_list():
        source, owner = entry.split("|", 1)
        if not source.startswith(prefix + "src/chrono_sensor/"):
            continue
        if source not in expected or expected[source] != owner:
            fail("Mixed or unexpected OptiX Sensor compile owner: " + entry)
        found[source] = owner
    if found != expected:
        fail("The OptiX Sensor profile omits an expected translation unit")
    definitions = ctx.attr.implementation[CcInfo].compilation_context.defines.to_list()
    for required in ["CHRONO_USE_CUDA", "USE_SENSOR_GLFW", "CH_API_COMPILE_SENSOR"]:
        if required not in definitions:
            fail("Missing retained OptiX consumer definition: " + required)
    for forbidden in ["USE_CUDA_NVRTC", "USE_SENSOR_NVDB", "CHRONO_SENSOR_VULKAN_RT_GPU_ENABLED"]:
        if forbidden in definitions:
            fail("Private or disabled Sensor definition leaked: " + forbidden)
    output = ctx.actions.declare_file(ctx.label.name + ".sh")
    ctx.actions.write(output, "#!/bin/sh\n# OptiX source partition and public definitions verified during analysis.\nexit 0\n", is_executable = True)
    return [DefaultInfo(executable = output)]

optix_ownership_test = rule(
    implementation = _ownership_impl,
    attrs = {
        "implementation": attr.label(mandatory = True, aspects = [native_closure_aspect], providers = [CcInfo]),
        "with_sph": attr.bool(default = False),
    },
    test = True,
)
