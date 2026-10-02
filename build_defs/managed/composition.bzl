"""Declared proxy composition before compiling one managed assembly."""

def _managed_proxy_set_impl(ctx):
    if len(ctx.attr.modules) != len(ctx.attr.proxies):
        fail("Every managed module needs exactly one declared proxy directory")
    output = ctx.actions.declare_directory(ctx.label.name + "/proxies")
    receipt = ctx.actions.declare_file(ctx.label.name + "/composition.json")
    arguments = ctx.actions.args()
    arguments.add_all(["--output", output.path, "--receipt", receipt.path])
    inputs = []
    for index, dependency in enumerate(ctx.attr.proxies):
        files = dependency[DefaultInfo].files.to_list()
        if len(files) != 1 or not files[0].is_directory:
            fail("Managed proxy input must be one generated source directory")
        inputs.append(files[0])
        arguments.add_all(["--module", ctx.attr.modules[index] + "=" + files[0].path])
    if ctx.file.approvals:
        inputs.append(ctx.file.approvals)
        arguments.add_all(["--approvals", ctx.file.approvals.path])
    ctx.actions.run(
        executable = ctx.executable._compose,
        inputs = inputs,
        outputs = [output, receipt],
        arguments = [arguments],
        mnemonic = "RobodynaManagedCompose",
        progress_message = "Compose managed proxy ownership for " + ctx.label.name,
    )
    return [DefaultInfo(files = depset([output])), OutputGroupInfo(receipt = depset([receipt]))]

managed_proxy_set = rule(
    implementation = _managed_proxy_set_impl,
    attrs = {
        "modules": attr.string_list(mandatory = True),
        "proxies": attr.label_list(mandatory = True),
        "approvals": attr.label(allow_single_file = True),
        "_compose": attr.label(default = "//tools/managed:compose", executable = True, cfg = "exec"),
    },
)
