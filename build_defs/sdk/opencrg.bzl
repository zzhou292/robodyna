"""Build the exact OpenCRG source revision named by the retained module recipe."""

load(":inspection.bzl", "file_sha256", "required_file")

def _opencrg_impl(ctx):
    if ctx.os.name != "linux" or ctx.os.arch not in ["amd64", "x86_64"]:
        fail("Initial OpenCRG profile requires Linux x86_64")
    pins = json.decode(ctx.read(ctx.attr.manifest))
    archive = ctx.os.environ.get("ROBODYNA_OPENCRG_ARCHIVE", "")
    if not archive.startswith("/"):
        fail("Set absolute ROBODYNA_OPENCRG_ARCHIVE for the reviewed local source archive")
    path = required_file(ctx, archive)
    if file_sha256(ctx, path) != pins["source"]["sha256"]:
        fail("OpenCRG archive differs from the reviewed immutable revision")
    ctx.extract(path, stripPrefix = pins["strip_prefix"])
    for name, identity in pins["files"].items():
        # These are outputs of this repository rule. The archive is watched;
        # explicitly watching files inside the repository being fetched is invalid.
        extracted = ctx.path(name)
        if not extracted.exists or extracted.is_dir or file_sha256(ctx, extracted) != identity["sha256"]:
            fail("Extracted OpenCRG source differs: " + name)
    ctx.file("BUILD.bazel", "\n".join([
        'load("@rules_cc//cc:defs.bzl", "cc_library")',
        'package(default_visibility = ["//visibility:public"])',
        'cc_library(name = "opencrg", srcs = %r, hdrs = %r,' % (pins["sources"], pins["headers"]),
        '    includes = ["inc"], copts = ["-O3", "-fPIC", "-Wall"], linkopts = ["-lm"])',
        'filegroup(name = "source_inputs", srcs = %r)' % sorted(pins["files"].keys()),
        'exports_files(["sdk.json", "LICENSE"])',
    ]) + "\n")
    ctx.file("sdk.json", json.encode_indent({
        "schema": "robodyna.local_opencrg_sdk.v1",
        "archive": archive,
        "source": pins["source"],
        "files": pins["files"],
        "scope": "Native Bazel C compilation from original source; no prebuilt OpenCRG or Chrono solver is imported.",
    }, indent = "  ") + "\n")

local_opencrg_sdk = repository_rule(
    implementation = _opencrg_impl,
    attrs = {"manifest": attr.label(default = Label("//build_defs/sdk:opencrg_pins.json"), allow_single_file = True)},
    environ = ["ROBODYNA_OPENCRG_ARCHIVE"],
    local = True,
)
