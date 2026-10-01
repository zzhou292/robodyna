"""Bazel-analysis assertions for the actual neutral header/link/source closure."""

load(":dependency_closure.bzl", "NativeClosure", "native_closure_aspect")
load(":neutral_sources.bzl", "NEUTRAL_HEADERS", "NEUTRAL_SOURCES", "NEUTRAL_TARGETS", "NEUTRAL_SOURCE_RELOCATIONS", "NEUTRAL_CANONICAL_HEADERS", "NEUTRAL_HEADER_TARGETS")
load(":native_sources.bzl", "NATIVE_SOURCE_GROUPS")
load(":visual_sources.bzl", "VISUAL_ADAPTER_SOURCES", "VISUAL_SOURCES", "VISUAL_TARGETS")

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
        closure = target[NativeClosure]
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
    for label in VISUAL_ADAPTER_SOURCES:
        original[label[2:].replace(":", "/")] = True
    composed_owners = dict(expected)
    for component, paths in VISUAL_SOURCES.items():
        for path in paths:
            composed_owners["src/compatibility/chrono/" + path] = str(Label(VISUAL_TARGETS[component]))
    compiled = {}
    for entry in ctx.attr.aggregate[NativeClosure].owners.to_list():
        path, owner = entry.split("|", 1)
        if path in compiled and compiled[path] != owner:
            fail("Translation unit compiled by more than one owner: " + path)
        compiled[path] = owner
        if path in composed_owners and owner != composed_owners[path]:
            fail("Aggregate retained an extracted translation unit: " + entry)
    if {path: True for path in compiled} != original:
        fail("Composed mechanics must compile the original 482 translation units and one declared FE adapter")
    executable = ctx.actions.declare_file(ctx.label.name + ".sh")
    ctx.actions.write(executable, "#!/bin/sh\n# All assertions executed during Bazel analysis.\nexit 0\n", is_executable = True)
    return [DefaultInfo(executable = executable)]

neutral_boundary_test = rule(
    implementation = _boundary_impl,
    attrs = {
        "components": attr.label_list(aspects = [native_closure_aspect], mandatory = True),
        "aggregate": attr.label(aspects = [native_closure_aspect], mandatory = True),
    },
    test = True,
)
