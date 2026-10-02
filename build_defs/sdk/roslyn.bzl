"""Explicit Microsoft C# compiler package; uses the existing Mono runtime."""

load(":inspection.bzl", "file_sha256", "required_file")

def _local_roslyn_impl(ctx):
    pins = json.decode(ctx.read(ctx.attr._pins))
    if pins["schema"] != "robodyna.roslyn_sdk_pins.v1":
        fail("Unsupported Roslyn SDK pin schema")
    root = ctx.os.environ.get(pins["environment"], "")
    if not root.startswith("/"):
        fail("Set absolute " + pins["environment"] + " to the extracted compiler SDK")
    extraction = json.decode(ctx.read(required_file(ctx, root + "/sdk.json")))
    if extraction["schema"] != "robodyna.zip_sdk.v1" or extraction["archive"]["sha256"] != pins["archive_sha256"]:
        fail("Compiler extraction is not the admitted official package")
    files = []
    for row in pins["files"]:
        path = required_file(ctx, root + "/" + row["path"])
        if file_sha256(ctx, path.realpath) != row["sha256"]:
            fail("Compiler input differs from the pinned package: " + row["path"])
        ctx.symlink(path.realpath, row["path"])
        files.append(row["path"])
    support = pins["runtime_overlay"]
    support_root = ctx.os.environ.get(support["environment"], "")
    if not support_root.startswith("/"):
        fail("Set absolute " + support["environment"] + " to the admitted Mono compiler-runtime overlay")
    support_receipt = json.decode(ctx.read(required_file(ctx, support_root + "/sdk.json")))
    if support_receipt["packages"][support["component"]]["sha256"] != support["archive_sha256"]:
        fail("Mono compiler-runtime overlay package differs from the admitted identity")
    for row in support["files"]:
        path = required_file(ctx, support_root + "/" + row["path"])
        if file_sha256(ctx, path.realpath) != row["sha256"]:
            fail("Mono compiler-runtime implementation differs from its pin: " + row["path"])
        ctx.symlink(path.realpath, row["output"])
        files.append(row["output"])
    ctx.file("BUILD.bazel", "package(default_visibility = [\"//visibility:public\"])\n" +
             "filegroup(name = \"compiler_files\", srcs = " + repr(files) + ")\n" +
             "exports_files([\"sdk.json\"] + " + repr(files) + ")\n")
    ctx.file("sdk.json", json.encode_indent({
        "schema": "robodyna.roslyn_sdk.v1", "root": root,
        "version": pins["version"], "language_version": pins["language_version"],
        "source_commit": pins["source_commit"], "archive_sha256": pins["archive_sha256"],
        "compiler": pins["compiler"], "files": pins["files"],
        "runtime_overlay": support,
        "scope": "Pinned compiler inputs only; actual language and native interop gates remain required",
    }) + "\n")

local_roslyn = repository_rule(
    implementation = _local_roslyn_impl,
    attrs = {"_pins": attr.label(default = Label("//build_defs/sdk:roslyn_pins.json"), allow_single_file = True)},
    environ = ["ROBODYNA_ROSLYN_ROOT", "ROBODYNA_MONO_ROSLYN_RUNTIME_ROOT"],
    local = True,
    configure = True,
)
