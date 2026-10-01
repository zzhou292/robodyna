"""Shared Bazel aspect for actual source, header and linker dependency closure."""

load("@rules_cc//cc/common:cc_info.bzl", "CcInfo")

NativeClosure = provider(fields = ["owners", "headers", "labels", "imports", "link_owners", "link_flags"])

def _closure_impl(target, ctx):
    dependencies = getattr(ctx.rule.attr, "deps", []) + getattr(ctx.rule.attr, "implementation_deps", [])
    children = [dep[NativeClosure] for dep in dependencies if NativeClosure in dep]
    sources = [file for file in getattr(ctx.rule.files, "srcs", []) if file.extension in ["cpp", "cc", "c", "cxx", "cu"]]
    headers = target[CcInfo].compilation_context.headers.to_list() if CcInfo in target else []
    link_owners = []
    link_flags = []
    if CcInfo in target:
        for entry in target[CcInfo].linking_context.linker_inputs.to_list():
            libraries = entry.libraries.to_list() if type(entry.libraries) == "depset" else entry.libraries
            if libraries:
                link_owners.append(str(entry.owner))
            flags = entry.user_link_flags.to_list() if type(entry.user_link_flags) == "depset" else entry.user_link_flags
            link_flags.extend(flags)
    return [NativeClosure(
        owners = depset([file.short_path + "|" + str(target.label) for file in sources], transitive = [c.owners for c in children]),
        headers = depset([file.short_path for file in headers], transitive = [c.headers for c in children]),
        labels = depset([str(target.label)], transitive = [c.labels for c in children]),
        imports = depset([str(target.label)] if ctx.rule.kind == "cc_import" else [], transitive = [c.imports for c in children]),
        link_owners = depset(link_owners, transitive = [c.link_owners for c in children]),
        link_flags = depset(link_flags, transitive = [c.link_flags for c in children]),
    )]

native_closure_aspect = aspect(implementation = _closure_impl, attr_aspects = ["deps", "implementation_deps"])
