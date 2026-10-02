"""Choose the retained explicit-archive driver path without mutable runfiles."""

def _configuration_impl(ctx):
    # All original mains have a supported external-FMU argument branch. The
    # export targets remain real and enabled; this is a driver-only selection.
    ctx.actions.expand_template(
        template = ctx.file.template,
        output = ctx.outputs.header,
        substitutions = {
            "@CHRONO_VEHICLE_FMU_DIR@": '#define CHRONO_VEHICLE_FMU_DIR std::string("./")\n#undef FMU_EXPORT_SUPPORT',
        },
    )
    return [DefaultInfo(files = depset([ctx.outputs.header]))]

vehicle_fmi_configuration = rule(
    implementation = _configuration_impl,
    attrs = {"template": attr.label(allow_single_file = True), "header": attr.output()},
)
