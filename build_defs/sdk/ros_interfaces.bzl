"""Source-built custom ROSIDL messages used by the existing dynamic ROS node."""

load(":inspection.bzl", "file_sha256", "required_file", "shared_library", "verify_file_hashes")

def _impl(ctx):
    if ctx.os.name != "linux" or ctx.os.arch not in ["amd64", "x86_64"]:
        fail("This custom ROSIDL SDK profile is Linux x86_64")
    root = ctx.os.environ.get("ROBODYNA_ROS_INTERFACES_ROOT", "")
    if not root.startswith("/"):
        fail("Set ROBODYNA_ROS_INTERFACES_ROOT to the reviewed source-built custom message installation")
    pins = json.decode(ctx.read(ctx.attr.pins))
    receipt = required_file(ctx, root + "/sdk.json")
    if file_sha256(ctx, receipt) != pins["sdk_receipt_sha256"]:
        fail("Custom ROSIDL source-build receipt changed")
    built = json.decode(ctx.read(receipt))
    if built.get("schema") != "robodyna.cmake_built_sdk.v1" or built.get("source_unchanged") != True:
        fail("Custom ROSIDL installation is not a completed source build")
    if built["source"]["commit"] != pins["source_commit"] or built["source"]["sha256"] != pins["source_archive_sha256"]:
        fail("Custom ROS message source revision differs")
    if len(built["installed_files"]) != pins["installed_file_count"]:
        fail("Custom ROSIDL installation inventory differs")
    hashes = {row["path"]: row["sha256"] for row in built["installed_files"] if "sha256" in row}
    verify_file_hashes(ctx, root, hashes)
    headers = [name for name in hashes if name.startswith("include/")]
    resources = [name for name in hashes if name.startswith("share/")]
    for name in headers + resources:
        ctx.symlink(ctx.path(root + "/" + name).realpath, name)
    for row in pins["messages"]:
        if hashes.get(row["path"]) != row["sha256"]:
            fail("Custom ROS schema identity changed: " + row["path"])
    middleware = json.decode(ctx.read(ctx.attr.ros_receipt))
    if middleware["schema"] != "robodyna.ros_sdk.v1":
        fail("Custom messages must reuse the admitted ROS middleware owner")
    platform = ["libstdc++.so.6", "libgcc_s.so.1", "libc.so.6", "libm.so.6"]
    if file_sha256(ctx, ctx.path(ctx.attr.license)) != pins["license_sha256"]:
        fail("Custom ROS interface source license changed")
    ctx.symlink(ctx.path(ctx.attr.license), "LICENSE")
    records = {}
    build = ['load("@rules_cc//cc:defs.bzl", "cc_import", "cc_library")',
             'package(default_visibility = ["//visibility:public"])']
    for name, pin in pins["libraries"].items():
        library = shared_library(ctx, root + "/" + pin["path"], name)
        if library.record["sha256"] != pin["sha256"] or library.record["needed"] != pin["needed"]:
            fail("Custom ROSIDL library identity changed: " + name)
        for needed in pin["needed"]:
            if needed not in middleware["libraries"] and needed not in platform:
                fail("Custom ROSIDL DSO has an unowned runtime dependency: " + needed)
        records[name] = library.record
        target = name[len("libchrono_ros_interfaces__"):-len(".so")]
        build.append('cc_import(name = %r, shared_library = %r)' % (target, library.output))
    build.extend([
        'cc_library(name="headers", hdrs=%r, includes=["include/chrono_ros_interfaces"], deps=["@ros_sdk//:typed_messages"])' % headers,
        'cc_library(name="typed_messages", deps=[":headers", ":rosidl_typesupport_cpp"], data=[":runtime"], linkopts=[%r])' % ("-Wl,-rpath," + root + "/lib"),
        'filegroup(name="runtime", srcs=%r)' % (resources + ["lib/" + name for name in records] + ["sdk.json", "LICENSE"]),
        'exports_files(["sdk.json"] + %r)' % resources,
    ])
    ctx.file("BUILD.bazel", "\n".join(build) + "\n")
    ctx.file("sdk.json", json.encode_indent({
        "schema": "robodyna.ros_interfaces_sdk.v1", "package": "chrono_ros_interfaces",
        "prefix": ".", "source": built["source"], "source_build_receipt_sha256": pins["sdk_receipt_sha256"],
        "messages": pins["messages"], "libraries": records,
        "scope": "All16 original message schemas; actual C++ introspection/FastDDS type support. C/Python/Rust artifacts remain preserved but outside this runtime profile.",
    }) + "\n")

local_ros_interfaces = repository_rule(
    implementation = _impl,
    attrs = {
        "pins": attr.label(default = Label("//build_defs/sdk:ros_interfaces_pins.json"), allow_single_file = True),
        "license": attr.label(default = Label("//build_defs/sdk/ros_interfaces:LICENSE"), allow_single_file = True),
        "ros_receipt": attr.label(default = Label("@ros_sdk//:sdk.json"), allow_single_file = True),
    },
    environ = ["ROBODYNA_ROS_INTERFACES_ROOT"], local = True, configure = True,
)
