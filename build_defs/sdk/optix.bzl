"""Admit pinned OptiX API, NPP and cuRAND headers without a second CUDA runtime."""

load(":inspection.bzl", "file_sha256", "required_file", "shared_library")

def _optix_impl(ctx):
    if ctx.os.name != "linux" or ctx.os.arch not in ["amd64", "x86_64"]:
        fail("The initial OptiX SDK profile is Linux x86_64")
    root = ctx.os.environ.get("ROBODYNA_OPTIX_ROOT", "")
    if not root.startswith("/"):
        fail("Set absolute ROBODYNA_OPTIX_ROOT for the declared OptiX/NPP/cuRAND SDK")
    expected = json.decode(ctx.read(ctx.attr.manifest))
    receipt_file = required_file(ctx, root + "/sdk.json")
    if file_sha256(ctx, receipt_file) != expected["sdk_receipt_sha256"]:
        fail("OptiX SDK receipt differs from its reviewed pin")
    receipt = json.decode(ctx.read(receipt_file))
    if receipt.get("schema") != "robodyna.optix_sdk.v1" or receipt["cuda_release"] != expected["cuda_release"]:
        fail("OptiX SDK release/schema differs")
    declared = {}
    for component, spec in expected["components"].items():
        record = receipt["components"][component]
        if record["sha256"] != spec["sha256"] or record["version"] != spec["version"]:
            fail("OptiX SDK component identity differs: " + component)
        for path, identity in record["files"].items():
            if not (path.startswith("include/") or path in ["LICENSE.txt", "license_info.txt"]):
                continue
            if "sha256" not in identity:
                fail("Unreviewed SDK header/license link: " + path)
            original = required_file(ctx, root + "/" + record["root"] + "/" + path)
            if file_sha256(ctx, original) != identity["sha256"]:
                fail("OptiX SDK header/license changed: " + path)
            destination = path if path.startswith("include/") else "licenses/" + component + "/" + path
            if destination in declared:
                fail("Overlapping OptiX/NPP/cuRAND header owner: " + destination)
            ctx.symlink(original, destination)
            declared[destination] = component
    if "#define OPTIX_VERSION 90100" not in ctx.read("include/optix.h"):
        fail("Expected the declared OptiX 9.1.0 header version")

    build = [
        'load("@rules_cc//cc:defs.bzl", "cc_import", "cc_library")',
        'package(default_visibility = ["//visibility:public"])',
        'cc_library(name = "headers", hdrs = glob(["include/**"]), includes = ["include"],',
        '    deps = ["@rules_cuda//cuda:runtime", "@cuda_math//:cccl_headers"])',
        'filegroup(name = "runtime_headers", srcs = glob(["include/**", "licenses/**"]))',
    ]
    libraries = {}
    for name, soname, deps in [
        ("nppc", "libnppc.so.13", []),
        ("nppig", "libnppig.so.13", [":nppc"]),
        ("nppidei", "libnppidei.so.13", [":nppc"]),
    ]:
        component = receipt["components"]["libnpp"]
        imported = shared_library(ctx, root + "/" + component["root"] + "/lib/lib" + name + ".so", soname)
        real_name = imported.record["resolved"].split("/")[-1]
        if imported.record["sha256"] != component["files"]["lib/" + real_name]["sha256"]:
            fail("NPP library differs from its pinned archive: " + name)
        libraries[name] = imported.record
        build.append('cc_import(name = %r, shared_library = %r)' % (name + "_binary", imported.output))
        build.append('cc_library(name = %r, deps = %r)' % (name, [":" + name + "_binary", ":headers", "@rules_cuda//cuda:runtime"] + deps))
    build.extend([
        'cc_library(name = "npp", deps = [":nppc", ":nppig", ":nppidei"])',
        'filegroup(name = "runtime", srcs = glob(["lib/*"]))',
        'exports_files(["sdk.json"])',
    ])
    ctx.file("BUILD.bazel", "\n".join(build) + "\n")
    ctx.file("sdk.json", json.encode_indent({
        "schema": "robodyna.local_optix_sdk.v1",
        "sdk_root": root,
        "sdk_receipt_sha256": expected["sdk_receipt_sha256"],
        "optix_version": "9.1.0",
        "optix_commit": expected["components"]["optix_headers"]["commit"],
        "libraries": libraries,
        "curand": "Original device-inline headers; no new host cuRAND runtime linked by this Sensor profile.",
        "runtime_owner": "@rules_cuda//cuda:runtime",
        "driver_runtime": "Installed NVIDIA driver including libnvoptix; no driver or link stub admitted as runtime data.",
        "scope": "Declared SDK inputs; native compilation, NVRTC and actual OptiX execution remain separate gates.",
    }, indent = "  ") + "\n")

local_optix_sdk = repository_rule(
    implementation = _optix_impl,
    attrs = {"manifest": attr.label(default = Label("//build_defs/sdk:optix_pins.json"), allow_single_file = True)},
    environ = ["ROBODYNA_OPTIX_ROOT"],
    local = True,
)
