"""Generate the original configuration template for one real Vulkan GPU profile."""

def _configuration_impl(ctx):
    output = ctx.actions.declare_file("generated/chrono_sensor/ChConfigSensor.h")
    ctx.actions.expand_template(
        template = ctx.file.template,
        output = output,
        substitutions = {
            "@CHRONO_HAS_OPTIX@": "#undef CHRONO_HAS_OPTIX",
            "@CHRONO_HAS_VULKAN_RT@": "#define CHRONO_HAS_VULKAN_RT",
            "@CHRONO_HAS_METAL_RT@": "#undef CHRONO_HAS_METAL_RT",
            "@CHRONO_HAS_SENSOR_RENDER@": "#define CHRONO_HAS_SENSOR_RENDER",
            "@VULKAN_SHADER_DIR@": "src/sensor/vulkan/shaders",
            "@SHADER_DIR@": "",
            "@CUDA_NVRTC_INCLUDE_LIST@": "{0}",
            "@CUDA_NVRTC_FLAG_LIST@": "{0}",
        },
    )
    return [DefaultInfo(files = depset([output]))]

sensor_configuration = rule(
    implementation = _configuration_impl,
    attrs = {"template": attr.label(mandatory = True, allow_single_file = True)},
)
