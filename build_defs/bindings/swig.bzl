"""One native wrapper generation action with explicit source and SDK inputs."""

def _swig_impl(ctx):
    wrapper = ctx.actions.declare_file(ctx.label.name + "/core_wrap.cpp")
    directors = ctx.actions.declare_file(ctx.label.name + "/core_wrap.h")
    report = ctx.actions.declare_file(ctx.label.name + "/generation.json")
    if ctx.attr.language == "python":
        proxy = ctx.actions.declare_file(ctx.label.name + "/core.py")
        proxy_dir = proxy.dirname
    else:
        proxy = ctx.actions.declare_directory(ctx.label.name + "/proxies")
        proxy_dir = proxy.path
    anchor_suffix = "src/chrono_swig/chrono_" + ctx.attr.language + "/ChModuleCore_" + ctx.attr.language + ".i"
    if not ctx.file.interface.path.endswith(anchor_suffix):
        fail("Expected the actual retained core interface")
    source_root = ctx.file.interface.path[:-len(anchor_suffix)] + "src"
    canonical = ctx.file.canonical_anchor.path
    canonical_suffix = "robodyna/mbd/RbBody.h"
    if not canonical.endswith(canonical_suffix):
        fail("Expected the canonical Robodyna include anchor")
    include_root = canonical[:-len(canonical_suffix)]
    views = ctx.files.declarations
    view_roots = []
    for view in views:
        suffix = "robodyna_swig/" + view.basename
        if not view.path.endswith(suffix):
            fail("Expected an authenticated declaration-view header")
        root = view.path[:-len(suffix)]
        if root not in view_roots:
            view_roots.append(root)
    args = ctx.actions.args()
    args.add_all(["--swig", ctx.executable._swig.path, "--language", ctx.attr.language,
                  "--interface", ctx.file.interface.path, "--wrapper", wrapper.path,
                  "--directors", directors.path, "--proxy-dir", proxy_dir, "--report", report.path,
                  "--include", source_root, "--include", include_root])
    for root in view_roots:
        args.add_all(["--include", root])
    ctx.actions.run(
        executable = ctx.executable._runner,
        tools = [ctx.attr._runner[DefaultInfo].files_to_run, ctx.attr._swig[DefaultInfo].files_to_run],
        inputs = depset([ctx.file.interface, ctx.file.canonical_anchor] + views,
                        transitive = [target[DefaultInfo].files for target in ctx.attr.sources]),
        arguments = [args],
        outputs = [wrapper, directors, proxy, report],
        mnemonic = "RobodynaSwigCore",
        progress_message = "Generate retained " + ctx.attr.language + " core wrappers (NumPy disabled)",
    )
    return [DefaultInfo(files = depset([wrapper, directors, proxy, report])),
            OutputGroupInfo(wrapper = depset([wrapper]), directors = depset([directors]),
                            proxy = depset([proxy]), report = depset([report]))]

swig_core = rule(
    implementation = _swig_impl,
    attrs = {
        "language": attr.string(values = ["python", "csharp"], mandatory = True),
        "interface": attr.label(allow_single_file = True, mandatory = True),
        "canonical_anchor": attr.label(allow_single_file = True, default = "//include/robodyna/mbd:RbBody.h"),
        "declarations": attr.label_list(allow_files = [".h"], default = ["//build_defs/bindings:body_header", "//build_defs/bindings:mesh_header", "//build_defs/bindings:system_header", "//build_defs/bindings:system_nsc_header", "//build_defs/bindings:system_smc_header"]),
        "sources": attr.label_list(allow_files = True),
        "_runner": attr.label(default = "//tools/bindings:run_swig", executable = True, cfg = "exec"),
        "_swig": attr.label(default = "@swig_sdk//:swig", executable = True, cfg = "exec"),
    },
)
