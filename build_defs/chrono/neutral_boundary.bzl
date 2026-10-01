"""Bazel-analysis assertions for the actual neutral header/link/source closure."""

load("@rules_cc//cc/common:cc_info.bzl", "CcInfo")
load(":neutral_sources.bzl", "NEUTRAL_HEADERS", "NEUTRAL_SOURCES", "NEUTRAL_TARGETS", "NEUTRAL_SOURCE_RELOCATIONS", "NEUTRAL_CANONICAL_HEADERS", "NEUTRAL_HEADER_TARGETS")
load(":native_sources.bzl", "NATIVE_SOURCE_GROUPS")

_Closure = provider(fields = ["owners", "headers", "labels", "imports", "link_owners", "link_flags"])

def _closure_impl(target, ctx):
    dependencies = getattr(ctx.rule.attr, "deps", []) + getattr(ctx.rule.attr, "implementation_deps", [])
    children = [dep[_Closure] for dep in dependencies if _Closure in dep]
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
    return [_Closure(
        owners = depset([file.short_path + "|" + str(target.label) for file in sources], transitive = [c.owners for c in children]),
        headers = depset([file.short_path for file in headers], transitive = [c.headers for c in children]),
        labels = depset([str(target.label)], transitive = [c.labels for c in children]),
        imports = depset([str(target.label)] if ctx.rule.kind == "cc_import" else [], transitive = [c.imports for c in children]),
        link_owners = depset(link_owners, transitive = [c.link_owners for c in children]),
        link_flags = depset(link_flags, transitive = [c.link_flags for c in children]),
    )]

_closure_aspect = aspect(implementation = _closure_impl, attr_aspects = ["deps", "implementation_deps"])

def _current_source(path):
    relocated = NEUTRAL_SOURCE_RELOCATIONS.get(path)
    return relocated[2:].replace(":", "/") if relocated else "src/compatibility/chrono/" + path

def _boundary_impl(ctx):
    expected = {}
    allowed_headers = {}
    for component, paths in NEUTRAL_SOURCES.items():
        for path in paths:
            expected[_current_source(path)] = str(Label(NEUTRAL_TARGETS[component]))
    for paths in NEUTRAL_HEADERS.values():
        for path in paths:
            allowed_headers["src/compatibility/chrono/" + path] = True
    for paths in NEUTRAL_CANONICAL_HEADERS.values():
        for path in paths:
            allowed_headers[path] = True
    # strip_include_prefix creates this declared symlink artifact in CcInfo.
    allowed_headers["include/robodyna/mechanics/_virtual_includes/inertia_headers/robodyna/mechanics/RbMassProperties.h"] = True
    allowed_labels = {str(Label(label)): True for label in NEUTRAL_TARGETS.values() + [
        "//src/core/configuration:host_headers", "//src/compatibility/chrono:neutral_include_root", "@eigen//:eigen",
    ]}
    for targets in NEUTRAL_HEADER_TARGETS.values():
        for label in targets:
            allowed_labels[str(Label(label))] = True
    actual = {}
    for target in ctx.attr.components:
        closure = target[_Closure]
        if closure.imports.to_list():
            fail("Neutral mechanics cannot import implementation binaries: " + str(closure.imports.to_list()))
        for label in closure.labels.to_list():
            if label not in allowed_labels:
                fail("Undeclared neutral dependency (aggregate/domain leakage): " + label)
        for owner in closure.link_owners.to_list():
            if owner not in allowed_labels:
                fail("Link implementation outside neutral ownership: " + owner)
        for flag in closure.link_flags.to_list():
            if flag != "-pthread":
                fail("Unreviewed neutral linker flag (possible undeclared binary): " + flag)
        for header in closure.headers.to_list():
            if header in allowed_headers or header.startswith("src/core/configuration/") or header.startswith("../eigen"):
                continue
            fail("Header outside the reviewed neutral closure: " + header)
        for entry in closure.owners.to_list():
            path, owner = entry.split("|", 1)
            if path not in expected or owner != expected[path]:
                fail("Incorrect neutral translation-unit owner: " + entry)
            actual[path] = owner
    if actual != expected:
        fail("Neutral closure does not contain the exact 21 reviewed translation units")
    # Check the composed aggregate as well: proving the lower libraries are
    # narrow would not detect a leftover second compilation in the aggregate.
    original = {_current_source(path): True for paths in NATIVE_SOURCE_GROUPS.values() for path in paths}
    compiled = {}
    for entry in ctx.attr.aggregate[_Closure].owners.to_list():
        path, owner = entry.split("|", 1)
        if path in compiled and compiled[path] != owner:
            fail("Translation unit compiled by more than one owner: " + path)
        compiled[path] = owner
        if path in expected and owner != expected[path]:
            fail("Aggregate retained an extracted translation unit: " + entry)
    if {path: True for path in compiled} != original:
        fail("Composed mechanics does not compile the exact original 482 translation units")
    executable = ctx.actions.declare_file(ctx.label.name + ".sh")
    ctx.actions.write(executable, "#!/bin/sh\n# All assertions executed during Bazel analysis.\nexit 0\n", is_executable = True)
    return [DefaultInfo(executable = executable)]

neutral_boundary_test = rule(
    implementation = _boundary_impl,
    attrs = {
        "components": attr.label_list(aspects = [_closure_aspect], mandatory = True),
        "aggregate": attr.label(aspects = [_closure_aspect], mandatory = True),
    },
    test = True,
)
