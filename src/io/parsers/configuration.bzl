"""Keep one declared parser configuration for the actually admitted adapters."""

def _configuration_impl(ctx):
    ctx.actions.expand_template(
        template = ctx.file.template,
        output = ctx.outputs.header,
        substitutions = {
            "@CHRONO_URDF@": "#define CHRONO_HAS_URDF",
            "@CHRONO_PYTHON@": "#define CHRONO_HAS_PYTHON",
            "@CHRONO_ROS@": "#undef CHRONO_HAS_ROS",
        },
    )
    return [DefaultInfo(files = depset([ctx.outputs.header]))]

parser_configuration = rule(
    implementation = _configuration_impl,
    attrs = {"template": attr.label(allow_single_file = True), "header": attr.output()},
)
