"""Declared SWIG-only views; these outputs are deliberately not CcInfo headers."""

def _body_view_impl(ctx):
    canonical_relative = "include/robodyna/mbd/RbBody.h"
    if not ctx.file.canonical.path.endswith(canonical_relative):
        fail("Body view must authenticate the canonical body header")
    root = ctx.file.canonical.path[:-len(canonical_relative)] or "."
    header = ctx.actions.declare_file("generated/robodyna_swig/BodyDeclarations.h")
    receipt = ctx.actions.declare_file("generated/robodyna_swig/BodyDeclarations.h.json")
    args = ctx.actions.args()
    args.add_all(["--root", root, "--ledger", ctx.file.ledger.path,
                  "--original-path", "src/compatibility/chrono/src/chrono/physics/ChBody.h",
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
        progress_message = "Authenticate canonical body and generate SWIG-only declarations",
    )
    return [DefaultInfo(files = depset([header, receipt])),
            OutputGroupInfo(header = depset([header]), receipt = depset([receipt]))]

body_declaration_view = rule(
    implementation = _body_view_impl,
    attrs = {
        "canonical": attr.label(allow_single_file = True, mandatory = True),
        "forwarder": attr.label(allow_single_file = True, mandatory = True),
        "ledger": attr.label(allow_single_file = True, mandatory = True),
        "original_sha256": attr.string(mandatory = True),
        "ledger_sha256": attr.string(mandatory = True),
        "_generator": attr.label(default = "//tools/bindings:declaration_view", executable = True, cfg = "exec"),
    },
)
