"""Generate the retained FMI export-availability header from its own template."""

def _configuration_impl(ctx):
    ctx.actions.expand_template(
        template = ctx.file.template,
        output = ctx.outputs.header,
        substitutions = {"@DEFINE_FMU_EXPORT_SUPPORT@": "#define FMU_EXPORT_SUPPORT"},
    )
    return [DefaultInfo(files = depset([ctx.outputs.header]))]

fmi_configuration = rule(
    implementation = _configuration_impl,
    attrs = {
        "template": attr.label(allow_single_file = True, mandatory = True),
        "header": attr.output(mandatory = True),
    },
)
