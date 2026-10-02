"""Explicit local serial HDF5 C/C++ SDK for hydrodynamic input files."""

load(":inspection.bzl", "file_sha256", "required_file", "shared_library")

def _impl(ctx):
    root = ctx.os.environ.get("ROBODYNA_HDF5_ROOT", "")
    if not root.startswith("/"):
        fail("Set ROBODYNA_HDF5_ROOT to the admitted serial HDF5 SDK usr directory")
    pins = json.decode(ctx.read(ctx.attr.pins))
    for relative, digest in pins["files"].items():
        path = required_file(ctx, root + "/" + relative)
        if file_sha256(ctx, path.realpath) != digest:
            fail("HDF5 SDK file differs: " + relative)
    headers = ctx.path(root + "/include/hdf5/serial")
    ctx.watch_tree(headers)
    ctx.symlink(headers, "include")
    ctx.symlink(root + "/share/doc/libhdf5-dev/copyright", "LICENSE")
    build = [
        'load("@rules_cc//cc:defs.bzl", "cc_import", "cc_library")',
        'package(default_visibility = ["//visibility:public"])',
    ]
    libraries = {}
    for name, soname in pins["libraries"].items():
        library = shared_library(ctx, root + "/lib/x86_64-linux-gnu/" + soname, soname)
        libraries[name] = library.record
        build.append('cc_import(name = %s, shared_library = %s)' % (repr(name), repr(library.output)))
    build += [
        'cc_library(name = "c", hdrs = glob(["include/*.h"]), includes = ["include"], deps = [":serial_binary"])',
        'cc_library(name = "cpp", hdrs = glob(["include/*.h"]), includes = ["include"], deps = [":c", ":cpp_binary"])',
        'filegroup(name = "runtime", srcs = glob(["lib/*"]))',
        'exports_files(["sdk.json", "LICENSE"])',
    ]
    ctx.file("BUILD.bazel", "\n".join(build) + "\n")
    ctx.file("sdk.json", json.encode_indent({"schema": "robodyna.hdf5_sdk.v1", "libraries": libraries,
        "platform_runtime": "OS C/C++ ABI, compression, curl and its TLS dependencies"}) + "\n")

local_hdf5 = repository_rule(
    implementation = _impl,
    attrs = {"pins": attr.label(default = "//build_defs/sdk:hdf5_pins.json", allow_single_file = True)},
    environ = ["ROBODYNA_HDF5_ROOT"],
    local = True,
    configure = True,
)
