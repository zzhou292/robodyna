"""Admit the pinned hydrodynamic dependency source; compile it inside Bazel."""

load(":inspection.bzl", "file_sha256", "required_file", "run_tool")

def _impl(ctx):
    root = ctx.os.environ.get("ROBODYNA_HYDROCHRONO_ROOT", "")
    if not root.startswith("/"):
        fail("Set ROBODYNA_HYDROCHRONO_ROOT to the pinned HydroChrono source checkout")
    pins = json.decode(ctx.read(ctx.attr.pins))
    revision = run_tool(ctx, "git", ["-C", root, "rev-parse", "HEAD"]).strip()
    if revision != pins["commit"]:
        fail("Hydrodynamic dependency revision differs from the retained gitlink")
    for relative, digest in pins["files"].items():
        path = required_file(ctx, root + "/" + relative)
        if file_sha256(ctx, path) != digest:
            fail("Hydrodynamic dependency source changed: " + relative)
        ctx.symlink(path, relative)
    ctx.file("BUILD.bazel", "\n".join([
        'load("@rules_cc//cc:defs.bzl", "cc_library")',
        'package(default_visibility = ["//visibility:public"])',
        'cc_library(name = "hydrodynamics", srcs = %s, hdrs = %s,' % (repr(pins["sources"]), repr(pins["headers"])),
        '    includes = ["include", "src"], copts = ["-O3", "-fPIC", "-w"],',
        '    deps = ["@eigen//:eigen", "@hdf5_sdk//:cpp", %s], alwayslink = True)' % repr(str(ctx.attr.core_headers)),
        'exports_files(["LICENSE", "source.json"])',
    ]) + "\n")
    ctx.file("source.json", json.encode_indent(pins) + "\n")

local_hydrochrono = repository_rule(
    implementation = _impl,
    attrs = {
        "pins": attr.label(default = "//build_defs/sdk:hydrochrono_pins.json", allow_single_file = True),
        # Its retained HDF5 reader uses ChMatrix aliases. Supply our actual
        # headers; the coupled owner links the one native mechanics backend.
        "core_headers": attr.label(default = "//src/compatibility/chrono:native_core_fea_headers"),
    },
    environ = ["ROBODYNA_HYDROCHRONO_ROOT"],
    local = True,
    configure = True,
)
