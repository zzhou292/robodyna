"""Workspace-local Irrlicht development SDK; no package installation on fetch."""

load(":inspection.bzl", "required_file", "shared_library")

def _impl(ctx):
    if ctx.os.name != "linux" or ctx.os.arch not in ["amd64", "x86_64"]:
        fail("This Irrlicht SDK profile is Linux x86_64")
    root = ctx.os.environ.get("ROBODYNA_IRRLICHT_ROOT", "")
    if not root.startswith("/"):
        fail("Set ROBODYNA_IRRLICHT_ROOT to the admitted SDK usr directory")
    config = required_file(ctx, root + "/include/irrlicht/IrrCompileConfig.h")
    if '#define IRRLICHT_SDK_VERSION "1.8.5"' not in ctx.read(config):
        fail("This Irrlicht SDK profile requires version 1.8.5")
    headers = ctx.path(root + "/include/irrlicht")
    ctx.watch_tree(headers)
    ctx.symlink(headers, "include/irrlicht")
    library = shared_library(ctx, root + "/lib/x86_64-linux-gnu/libIrrlicht.so", "libIrrlicht.so.1.8")
    ctx.file("BUILD.bazel", "\n".join([
        'load("@rules_cc//cc:defs.bzl", "cc_import", "cc_library")',
        'package(default_visibility = ["//visibility:public"])',
        'cc_import(name = "binary", shared_library = %s)' % repr(library.output),
        'cc_library(name = "sdk", hdrs = glob(["include/irrlicht/**/*.h"]), includes = ["include/irrlicht"], deps = [":binary"])',
        'filegroup(name = "runtime", srcs = [%s])' % repr(library.output),
        'exports_files(["sdk.json"])',
    ]) + "\n")
    ctx.file("sdk.json", json.encode_indent({
        "schema": "robodyna.local_irrlicht_sdk.v1",
        "sdk_root": root,
        "version": "1.8.5",
        "library": library.record,
        "platform_prerequisites": "OS C/C++ ABI and the recorded OpenGL/X11/image-codec runtime dependencies",
    }) + "\n")

local_irrlicht = repository_rule(
    implementation = _impl,
    environ = ["ROBODYNA_IRRLICHT_ROOT"],
    local = True,
    configure = True,
)
