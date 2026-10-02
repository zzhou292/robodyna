"""Standalone FastDDS2.4/FastCDR1.0 SDK matching the retained SynChrono ABI."""

load(":inspection.bzl", "file_sha256", "required_file", "shared_library", "verify_file_hashes")

def _impl(ctx):
    root = ctx.os.environ.get("ROBODYNA_FASTDDS_ROOT", "")
    if not root.startswith("/"):
        fail("Set ROBODYNA_FASTDDS_ROOT to the admitted standalone three-component SDK root")
    if ctx.os.name != "linux" or ctx.os.arch not in ["amd64", "x86_64"]:
        fail("The current FastDDS SDK profile is Linux x86_64")
    pins = json.decode(ctx.read(ctx.attr.pins))
    records = {}
    headers = {}
    for name, component in pins["components"].items():
        directory = root + "/" + name
        receipt = required_file(ctx, directory + "/sdk.json")
        if file_sha256(ctx, receipt) != component["receipt_sha256"]:
            fail("Standalone FastDDS build receipt changed: " + name)
        verify_file_hashes(ctx, directory, component["files"])
        headers[name] = [name + "/" + path for path in component["files"] if path.startswith("include/")]
        for path in component["files"]:
            if path.startswith("include/"):
                ctx.symlink(directory + "/" + path, name + "/" + path)
            if path.endswith("/LICENSE"):
                ctx.symlink(directory + "/" + path, "licenses/" + name + ".txt")
        for pin in component["libraries"]:
            requested = directory + "/" + pin["path"]
            if pin["path"].endswith(".a"):
                path = required_file(ctx, requested)
                if file_sha256(ctx, path) != pin["sha256"]:
                    fail("Standalone foonathan archive differs")
                ctx.symlink(path, "lib/libfoonathan_memory.a")
                records[name] = {"requested": requested, "sha256": pin["sha256"], "linkage": "static PIC"}
            else:
                value = shared_library(ctx, requested, pin["soname"])
                if value.record["sha256"] != pin["sha256"] or value.record["needed"] != pin["needed"]:
                    fail("Standalone DDS ELF identity changed: " + name)
                records[name] = value.record
    ssl_pin = pins["openssl_ssl"]
    ssl = shared_library(ctx, ssl_pin["path"], ssl_pin["soname"])
    if ssl.record["sha256"] != ssl_pin["sha256"] or ssl.record["needed"] != ssl_pin["needed"]:
        fail("FastDDS OpenSSL SSL runtime differs from its built SDK")
    crypto = json.decode(ctx.read(ctx.attr.openssl_receipt))
    if file_sha256(ctx, required_file(ctx, crypto["library_resolved"])) != pins["openssl_crypto_sha256"]:
        fail("FastDDS must reuse the matching existing OpenSSL Crypto owner")
    interface = pins["interface"]
    ctx.file("BUILD.bazel", "\n".join([
        'load("@rules_cc//cc:defs.bzl", "cc_import", "cc_library")',
        'package(default_visibility = ["//visibility:public"])',
        'cc_import(name = "foonathan_binary", static_library = "lib/libfoonathan_memory.a")',
        'cc_library(name = "foonathan", hdrs = ' + repr(headers["foonathan"]) + ', includes = ["foonathan/include/foonathan_memory"],',
        '    defines = ' + repr(interface["foonathan_defines"]) + ', deps = [":foonathan_binary"])',
        'cc_import(name = "fastcdr_binary", shared_library = "lib/libfastcdr.so.1")',
        'cc_library(name = "fastcdr", hdrs = ' + repr(headers["fastcdr"]) + ', includes = ["fastcdr/include"], deps = [":fastcdr_binary"])',
        'cc_import(name = "ssl_binary", shared_library = "lib/libssl.so.3")',
        'cc_import(name = "dds_binary", shared_library = "lib/libfastrtps.so.2.4")',
        'cc_library(name = "dds", hdrs = ' + repr(headers["fastdds"]) + ', includes = ["fastdds/include"],',
        '    deps = [":dds_binary", ":fastcdr", ":foonathan", ":ssl_binary", "@openssl//:crypto"],',
        '    data = [":runtime"], linkopts = ' + repr(interface["fastdds_system_linkopts"]) + ')',
        'filegroup(name = "runtime", srcs = glob(["lib/*.so*", "licenses/*"]) + ["sdk.json"])',
        'exports_files(["sdk.json"])',
    ]) + "\n")
    ctx.file("sdk.json", json.encode_indent({
        "schema": "robodyna.fastdds_sdk.v1", "root": root, "libraries": records,
        "openssl_ssl": ssl.record, "openssl_crypto": crypto["library_resolved"], "interface": interface,
        "scope": "Pinned standalone FastDDS2.4 ABI; ROS2 FastDDS2.6 is intentionally a separate process profile",
    }) + "\n")

local_fastdds = repository_rule(
    implementation = _impl,
    attrs = {
        "pins": attr.label(default = "//build_defs/sdk:fastdds_pins.json", allow_single_file = True),
        "openssl_receipt": attr.label(default = "@openssl//:sdk.json", allow_single_file = True),
    },
    environ = ["ROBODYNA_FASTDDS_ROOT"],
    local = True,
    configure = True,
)
