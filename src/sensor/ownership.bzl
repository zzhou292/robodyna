"""Verify the real Vulkan Sensor implementation and reused third-party owners."""

load("@rules_cc//cc/common:cc_info.bzl", "CcInfo")
load("//build_defs/chrono:dependency_closure.bzl", "NativeClosure", "native_closure_aspect")
load(":sources.bzl", "SENSOR_GROUPS")

def _ownership_impl(ctx):
    prefix = "src/compatibility/chrono/"
    expected = {prefix + path: str(Label("//src/sensor:sensor")) for group in SENSOR_GROUPS.values() for path in group["sources"]}
    for source in ["stb_image.cpp", "stb_image_write.cpp"]:
        expected[prefix + "src/chrono_thirdparty/stb/" + source] = str(Label("//src/compatibility/chrono:native_stb"))
    expected[prefix + "src/chrono_thirdparty/tinyobjloader/tiny_obj_loader.cc"] = str(Label("//src/compatibility/chrono:native_core_fea_bundled_collision"))
    found = {}
    for entry in ctx.attr.implementation[NativeClosure].owners.to_list():
        source, owner = entry.split("|", 1)
        if source.startswith(prefix + "src/chrono_sensor/") or source.startswith(prefix + "src/chrono_thirdparty/stb/") or source.startswith(prefix + "src/chrono_thirdparty/tinyobjloader/"):
            if source not in expected or expected[source] != owner:
                fail("Unexpected or duplicate Sensor/third-party compile owner: " + entry)
            found[source] = owner
    if found != expected:
        fail("Sensor Vulkan profile omits a retained implementation")
    definitions = ctx.attr.implementation[CcInfo].compilation_context.defines.to_list()
    for required in ["CH_API_COMPILE_SENSOR", "USE_SENSOR_GLFW"]:
        if required not in definitions:
            fail("Sensor lost a retained public CMake definition: " + required)
    if ("CHRONO_USE_CUDA" in definitions) != ctx.attr.with_sph:
        fail("Vulkan Sensor must expose CUDA headers only when its declared SPH dependency requires them")
    for forbidden in ["CHRONO_SENSOR_VULKAN_RT_GPU_ENABLED", "USE_CUDA_NVRTC"]:
        if forbidden in definitions:
            fail("Unexpected definition in Sensor Vulkan consumer profile: " + forbidden)
    executable = ctx.actions.declare_file(ctx.label.name + ".sh")
    ctx.actions.write(executable, "#!/bin/sh\n# Actual Vulkan Sensor source owners and definitions verified in analysis.\nexit 0\n", is_executable = True)
    return [DefaultInfo(executable = executable)]

sensor_ownership_test = rule(
    implementation = _ownership_impl,
    attrs = {
        "implementation": attr.label(mandatory = True, aspects = [native_closure_aspect], providers = [CcInfo]),
        "with_sph": attr.bool(default = False),
    },
    test = True,
)
