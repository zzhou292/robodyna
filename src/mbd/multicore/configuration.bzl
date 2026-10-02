"""Retain the Multicore module's double-precision, CPU-only CMake profile."""

def _configuration_impl(ctx):
    ctx.actions.expand_template(
        template = ctx.file.template,
        output = ctx.outputs.header,
        substitutions = {
            "@CHRONO_MULTICORE_USE_DOUBLE@": "#define CHRONO_MULTICORE_USE_DOUBLE",
            "@CHRONO_MULTICORE_USE_CUDA@": "#undef CHRONO_MULTICORE_USE_CUDA",
        },
    )
    return [DefaultInfo(files = depset([ctx.outputs.header]))]

multicore_configuration = rule(
    implementation = _configuration_impl,
    attrs = {
        "template": attr.label(allow_single_file = True, mandatory = True),
        "header": attr.output(mandatory = True),
    },
)
