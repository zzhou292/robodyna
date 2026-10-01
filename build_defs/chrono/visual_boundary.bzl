"""Assert actual generic object/visual header, source and link ownership."""

load(":dependency_closure.bzl", "NativeClosure", "native_closure_aspect")
load(":neutral_sources.bzl", "NEUTRAL_HEADERS", "NEUTRAL_SOURCES")
load(":visual_sources.bzl", "VISUAL_EXTRA_HEADERS", "VISUAL_HEADERS", "VISUAL_SOURCES", "VISUAL_TARGETS")

def _boundary_impl(ctx):
    prefix = "src/compatibility/chrono/"
    expected = {prefix + path: str(Label("//src/core:foundation")) for path in NEUTRAL_SOURCES["foundation"]}
    allowed_headers = {prefix + path: True for path in NEUTRAL_HEADERS["foundation"] + VISUAL_EXTRA_HEADERS}
    for component, paths in VISUAL_SOURCES.items():
        for path in paths:
            expected[prefix + path] = str(Label(VISUAL_TARGETS[component]))
    for paths in VISUAL_HEADERS.values():
        for path in paths:
            allowed_headers[prefix + path] = True
    allowed_labels = {str(Label(label)): True for label in VISUAL_TARGETS.values() + [
        "//src/core:foundation", "//src/core/vectors:vector2",
        "//src/core/configuration:host_headers", "//src/compatibility/chrono:neutral_include_root", "@eigen//:eigen",
    ]}
    closure = ctx.attr.component[NativeClosure]
    if closure.imports.to_list():
        fail("Generic visuals cannot import implementation binaries")
    for label in closure.labels.to_list() + closure.link_owners.to_list():
        if label not in allowed_labels:
            fail("Generic visual dependency leaks an unqualified owner: " + label)
    for flag in closure.link_flags.to_list():
        if flag != "-pthread":
            fail("Unreviewed visual linker flag: " + flag)
    for header in closure.headers.to_list():
        if header in allowed_headers or header.startswith("src/core/configuration/") or header.startswith("../eigen"):
            continue
        fail("Header outside generic visual closure: " + header)
    actual = {}
    for entry in closure.owners.to_list():
        path, owner = entry.split("|", 1)
        if path not in expected or owner != expected[path]:
            fail("Incorrect generic visual translation-unit owner: " + entry)
        actual[path] = owner
    if actual != expected:
        fail("Generic visuals must compile the 9 reviewed sources plus the 16-source foundation")
    executable = ctx.actions.declare_file(ctx.label.name + ".sh")
    ctx.actions.write(executable, "#!/bin/sh\n# Boundary assertions ran during Bazel analysis.\nexit 0\n", is_executable = True)
    return [DefaultInfo(executable = executable)]

visual_boundary_test = rule(
    implementation = _boundary_impl,
    attrs = {"component": attr.label(aspects = [native_closure_aspect], mandatory = True)},
    test = True,
)
