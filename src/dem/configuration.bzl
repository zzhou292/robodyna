"""Generate only the existing DEM-specific configuration header."""

def _dem_configuration_impl(ctx):
    # The owning CMake file uses configure_file(@ONLY), and this template has no
    # substitutions. Copying it preserves the inherited main-config include.
    ctx.actions.expand_template(
        template = ctx.file.template,
        output = ctx.outputs.header,
        substitutions = {},
    )
    return [DefaultInfo(files = depset([ctx.outputs.header]))]

dem_configuration = rule(
    implementation = _dem_configuration_impl,
    attrs = {
        "template": attr.label(allow_single_file = True, mandatory = True),
        "header": attr.output(mandatory = True),
    },
)
