"""Compile public headers first, then check the opposite legacy include order."""

load("@rules_cc//cc:defs.bzl", "cc_library")
load(":header_pairs.bzl", "HEADER_PAIRS")

def _sources_impl(ctx):
    outputs = []
    for index, pair in enumerate(HEADER_PAIRS):
        if pair[0] not in ctx.attr.headers:
            continue
        for order, headers in [("public_first", pair), ("legacy_first", (pair[1], pair[0]))]:
            source = ctx.actions.declare_file("%s/%s_%s.cpp" % (ctx.label.name, index, order))
            # No PCH or convenience header may precede the header under test.
            ctx.actions.write(source, "\n".join(["#include \"%s\"" % path for path in headers]) + "\n")
            outputs.append(source)
    return [DefaultInfo(files = depset(outputs))]

_header_sources = rule(
    implementation = _sources_impl,
    attrs = {"headers": attr.string_list(mandatory = True)},
)

def header_compilation(name, deps, headers):
    """Make missing includes and old/new include-order conflicts compile errors."""
    _header_sources(name = name + "_sources", headers = headers)
    cc_library(
        name = name,
        srcs = [":" + name + "_sources"],
        deps = deps,
        tags = ["manual", "cpu-only", "public-api"],
    )
