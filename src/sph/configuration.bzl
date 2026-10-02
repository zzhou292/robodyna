"""Retained CMake-default SPH precision and optional reconstruction profile."""

def _configuration_impl(ctx):
    ctx.actions.expand_template(
        template = ctx.file.template,
        output = ctx.outputs.header,
        substitutions = {
            # CMake leaves this substitution empty when CH_USE_SPH_DOUBLE=OFF.
            "@CHRONO_SPH_USE_DOUBLE@": "",
            "@CHRONO_HAS_SPLASHSURF@": "#undef CHRONO_HAS_SPLASHSURF",
            "@CHRONO_SPLASHSURF_EXECUTABLE@": "#undef SPLASHSURF_EXECUTABLE",
        },
    )
    return [DefaultInfo(files = depset([ctx.outputs.header]))]

sph_configuration = rule(
    implementation = _configuration_impl,
    attrs = {
        "template": attr.label(allow_single_file = True, mandatory = True),
        "header": attr.output(mandatory = True),
    },
)
