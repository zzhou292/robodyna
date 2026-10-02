"""Managed IL compilation against the existing, separately owned native DSO."""

ManagedAssemblyInfo = provider(fields = ["assembly", "references"])

def _managed_assembly_impl(ctx):
    expected_suffix = ".dll" if ctx.attr.kind == "library" else ".exe"
    if not ctx.attr.assembly_name.startswith("Robodyna.") or not ctx.attr.assembly_name.endswith(expected_suffix) or "/" in ctx.attr.assembly_name:
        fail("Use a Robodyna.* managed assembly name with the matching extension")
    assembly = ctx.actions.declare_file(ctx.attr.assembly_name)
    receipt = ctx.actions.declare_file(ctx.attr.assembly_name + ".compilation.json")
    references = depset(transitive = [dep[ManagedAssemblyInfo].references for dep in ctx.attr.deps])
    args = ctx.actions.args()
    args.add_all(["--mono", ctx.executable._mono.path, "--compiler", ctx.file._compiler.path,
                  "--kind", ctx.attr.kind, "--output", assembly.path, "--receipt", receipt.path])
    for source in ctx.files.srcs:
        args.add_all(["--source", source.path])
    for reference in ctx.files._references + references.to_list():
        args.add_all(["--reference", reference.path])
    for definition in ctx.attr.defines:
        args.add_all(["--define", definition])
    globals_files = []
    if ctx.file.globals_template:
        args.add_all(["--globals", ctx.file.globals_template.path])
        globals_files.append(ctx.file.globals_template)
    ctx.actions.run(
        executable = ctx.executable._compile,
        tools = [ctx.attr._compile[DefaultInfo].files_to_run, ctx.attr._mono[DefaultInfo].files_to_run],
        inputs = depset(ctx.files.srcs + ctx.files._references + ctx.files._compiler_runtime + [ctx.file._compiler] + globals_files,
                        transitive = [references]),
        outputs = [assembly, receipt],
        arguments = [args],
        mnemonic = "RobodynaManagedCompile",
        progress_message = "Compile " + ctx.attr.assembly_name + " using declared Mono/net472 references",
    )
    runfiles = ctx.runfiles(files = [assembly, receipt] + references.to_list())
    for dependency in ctx.attr.deps + ctx.attr.native_libraries:
        runfiles = runfiles.merge(dependency[DefaultInfo].default_runfiles)
        runfiles = runfiles.merge(ctx.runfiles(transitive_files = dependency[DefaultInfo].files))
    return [DefaultInfo(files = depset([assembly]), runfiles = runfiles),
            ManagedAssemblyInfo(assembly = assembly, references = depset([assembly], transitive = [references])),
            OutputGroupInfo(receipt = depset([receipt]))]

managed_assembly = rule(
    implementation = _managed_assembly_impl,
    attrs = {
        "assembly_name": attr.string(mandatory = True),
        "kind": attr.string(values = ["library", "exe"], default = "library"),
        "srcs": attr.label_list(allow_files = True, mandatory = True),
        "deps": attr.label_list(providers = [ManagedAssemblyInfo]),
        "defines": attr.string_list(),
        "native_libraries": attr.label_list(),
        "globals_template": attr.label(allow_single_file = True),
        "_references": attr.label(default = "@mono_sdk//:references"),
        "_compiler": attr.label(default = "@roslyn_sdk//:tasks/net472/csc.exe", allow_single_file = True),
        "_compiler_runtime": attr.label(default = "@roslyn_sdk//:compiler_files"),
        "_mono": attr.label(default = "@mono_sdk//:mono", executable = True, cfg = "exec"),
        "_compile": attr.label(default = "//tools/managed:compile", executable = True, cfg = "exec"),
    },
)
