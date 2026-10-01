"""Declared SWIG-only views; these outputs are deliberately not CcInfo headers."""

def _view_impl(ctx):
    canonical_relative = ctx.attr.canonical_path
    if not canonical_relative.startswith("include/robodyna/") or not ctx.file.canonical.path.endswith(canonical_relative):
        fail("Declaration view must authenticate its declared canonical header")
    if "/" in ctx.attr.view_name or not ctx.attr.view_name.endswith(".h"):
        fail("Declaration view name must be one header filename")
    root = ctx.file.canonical.path[:-len(canonical_relative)] or "."
    header = ctx.actions.declare_file("generated/robodyna_swig/" + ctx.attr.view_name)
    receipt = ctx.actions.declare_file("generated/robodyna_swig/" + ctx.attr.view_name + ".json")
    args = ctx.actions.args()
    args.add_all(["--root", root, "--ledger", ctx.file.ledger.path,
                  "--original-path", ctx.attr.original_path,
                  "--expected-original-sha256", ctx.attr.original_sha256,
                  "--expected-ledger-sha256", ctx.attr.ledger_sha256,
                  "--output", header.path])
    ctx.actions.run(
        executable = ctx.executable._generator,
        tools = [ctx.attr._generator[DefaultInfo].files_to_run],
        arguments = [args],
        inputs = [ctx.file.canonical, ctx.file.forwarder, ctx.file.ledger],
        outputs = [header, receipt],
        mnemonic = "RobodynaSwigDeclaration",
        progress_message = "Authenticate canonical source and generate " + ctx.attr.view_name,
    )
    return [DefaultInfo(files = depset([header, receipt])),
            OutputGroupInfo(header = depset([header]), receipt = depset([receipt]))]

declaration_view = rule(
    implementation = _view_impl,
    attrs = {
        "canonical": attr.label(allow_single_file = True, mandatory = True),
        "canonical_path": attr.string(mandatory = True),
        "forwarder": attr.label(allow_single_file = True, mandatory = True),
        "original_path": attr.string(mandatory = True),
        "view_name": attr.string(mandatory = True),
        "ledger": attr.label(allow_single_file = True, mandatory = True),
        "original_sha256": attr.string(mandatory = True),
        "ledger_sha256": attr.string(mandatory = True),
        "_generator": attr.label(default = "//tools/bindings:declaration_view", executable = True, cfg = "exec"),
    },
)
