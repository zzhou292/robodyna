"""Build-wide capabilities that affect inherited interfaces or compilation.

These flags are intentionally not translation-unit availability macros. Every
producer and consumer in one configured target graph sees the same profile.
"""

def _bool_capability_impl(ctx):
    return []

bool_capability = rule(
    implementation = _bool_capability_impl,
    build_setting = config.bool(flag = True),
)

def _choice_capability_impl(ctx):
    if ctx.build_setting_value not in ctx.attr.values:
        fail("Unsupported capability value: " + ctx.build_setting_value)
    return []

choice_capability = rule(
    implementation = _choice_capability_impl,
    build_setting = config.string(flag = True),
    attrs = {"values": attr.string_list(mandatory = True)},
)

FSI_SPH_ONLY = select({
    "//build_defs/features:fsi_sph_enabled": [],
    "//conditions:default": ["@platforms//:incompatible"],
})

OPENCRG_ONLY = select({
    "//build_defs/features:opencrg_enabled": [],
    "//conditions:default": ["@platforms//:incompatible"],
})

SENSOR_VULKAN_ONLY = select({
    "//build_defs/features:sensor_vulkan": [],
    "//conditions:default": ["@platforms//:incompatible"],
})

SENSOR_OPTIX_ONLY = select({
    "//build_defs/features:sensor_optix": [],
    "//conditions:default": ["@platforms//:incompatible"],
})

# Existing foreign CMake and language-binding outputs have only been qualified
# with the baseline class layout. Explicitly reject a mixed profile until their
# generation/configuration and runtime gates are admitted together.
MULTICORE_ONLY = select({
    "//build_defs/features:multicore_enabled": [],
    "//conditions:default": ["@platforms//:incompatible"],
})

YAML_ONLY = select({
    "//build_defs/features:yaml_enabled": [],
    "//conditions:default": ["@platforms//:incompatible"],
})

BASELINE_ABI_ONLY = select({
    "//build_defs/features:yaml_enabled": ["@platforms//:incompatible"],
    "//build_defs/features:vsg_enabled": ["@platforms//:incompatible"],
    "//build_defs/features:fsi_tdpf_enabled": ["@platforms//:incompatible"],
    "//build_defs/features:fsi_sph_enabled": ["@platforms//:incompatible"],
    "//build_defs/features:sensor_optix": ["@platforms//:incompatible"],
    "//build_defs/features:fea_multiphysics_enabled": ["@platforms//:incompatible"],
    "//build_defs/features:multicore_enabled": ["@platforms//:incompatible"],
    "//conditions:default": [],
})
