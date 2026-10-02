"""One consistent MPI and FastDDS profile for all distributed-state owners."""

def _impl(ctx):
    ctx.actions.expand_template(
        template = ctx.file.template,
        output = ctx.outputs.header,
        substitutions = {"@CHRONO_SYNCHRONO_USE_FASTDDS@": "#define CHRONO_SYNCHRONO_USE_FASTDDS"},
    )
    return [DefaultInfo(files = depset([ctx.outputs.header]))]

distributed_configuration = rule(
    implementation = _impl,
    attrs = {
        "template": attr.label(mandatory = True, allow_single_file = True),
        "header": attr.output(mandatory = True),
    },
)
