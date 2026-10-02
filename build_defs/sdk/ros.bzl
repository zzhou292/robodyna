"""Pinned ROS2/DDS SDK inputs; the simulation library never links this provider."""

load(":inspection.bzl", "required_file", "run_tool", "shared_library")

def _files(ctx, root, entries):
    """Authenticate headers/resources in bounded batches using the shared tool."""
    paths = []
    for row in entries:
        paths.append(required_file(ctx, root + "/" + row["path"]))
    for start in range(0, len(paths), 256):
        selected = paths[start:start + 256]
        hashes = run_tool(ctx, "sha256sum", [str(path.realpath) for path in selected]).splitlines()
        if len(hashes) != len(selected):
            fail("ROS SDK checksum output is incomplete")
        for index, line in enumerate(hashes):
            row = entries[start + index]
            if line.split(" ")[0] != row["sha256"]:
                fail("ROS SDK input differs from the pinned package: " + row["path"])
    for index, row in enumerate(entries):
        ctx.symlink(paths[index].realpath, row["path"])
    return [row["path"] for row in entries]

def _local_ros_impl(ctx):
    if ctx.os.name != "linux" or ctx.os.arch not in ["amd64", "x86_64"]:
        fail("The admitted ROS2 SDK profile requires Linux x86_64")
    pins = json.decode(ctx.read(ctx.attr._pins))
    if pins["schema"] != "robodyna.ros_sdk_pins.v1":
        fail("Unsupported ROS2 SDK pin schema")
    for definition in pins["definitions"]:
        if any([token in definition for token in ["$", "\\", "<", ">", ";", " ", "\n"]]):
            fail("ROS SDK definition must be resolved for the admitted target profile: " + definition)
    root = ctx.os.environ.get(pins["environment"], "")
    if not root.startswith("/"):
        fail("Set absolute " + pins["environment"] + " to the extracted ROS2 SDK")
    extraction = json.decode(ctx.read(required_file(ctx, root + "/sdk.json")))
    actual_packages = {name: row["sha256"] for name, row in extraction["packages"].items()}
    if actual_packages != pins["package_hashes"]:
        fail("ROS SDK extraction package identities differ from the admitted closure")
    headers = _files(ctx, root, pins["headers"])
    resources = _files(ctx, root, pins["resources"])
    build = ['load("@rules_cc//cc:defs.bzl", "cc_import", "cc_library")',
             'package(default_visibility = ["//visibility:public"])']
    runtime = []
    links = []
    imports = {}
    records = {}
    for index, row in enumerate(pins["libraries"]):
        library = shared_library(ctx, root + "/" + row["path"], row["soname"])
        if library.record["sha256"] != row["sha256"] or library.record["needed"] != row["needed"]:
            fail("ROS SDK binary differs from its admitted package: " + row["path"])
        name = "library_" + str(index)
        imports[row["soname"]] = ":" + name
        records[row["soname"]] = library.record
        # Preserve ament's real prefix/lib layout for plugin and type-support
        # discovery as well as the declared cc_import runtime artifact.
        ctx.symlink(ctx.path(root + "/" + row["path"]).realpath, row["path"])
        runtime.extend([library.output, row["path"]])
        directory = row["path"].rsplit("/", 1)[0]
        soname_path = directory + "/" + row["soname"]
        if soname_path != row["path"]:
            ctx.symlink(ctx.path(root + "/" + row["path"]).realpath, soname_path)
            runtime.append(soname_path)
        build.append('cc_import(name = %s, shared_library = %s)' % (repr(name), repr(library.output)))
        if row["link"]:
            links.append(":" + name)
    ros_prefix = root + "/" + pins["prefix"]
    directories = [ros_prefix + "/lib", root + "/usr/lib/x86_64-linux-gnu"]
    rpaths = ["-Wl,-rpath," + path for path in directories]
    build.extend([
        'cc_library(name = "headers", hdrs = %s, includes = %s)' % (repr(headers), repr(pins["include_directories"])),
        'filegroup(name = "runtime", srcs = %s)' % repr(runtime + resources),
        'cc_library(name = "node_sdk", deps = [":headers"] + %s, defines = %s, data = [":runtime"], linkopts = %s)' %
        (repr(links), repr(pins["definitions"]), repr(pins["platform_linkopts"] + rpaths)),
        # The production node is schema-driven. Only typed peers/tests need
        # these additional message include roots and C++ type-support symbols.
        'cc_library(name = "typed_messages", deps = [":node_sdk"] + %s, includes = %s)' %
        (repr([imports[name] for name in pins["typed_messages"]["sonames"]]), repr(pins["typed_messages"]["include_directories"])),
        'exports_files(["sdk.json"] + %s)' % repr(runtime + resources),
    ])
    ctx.file("BUILD.bazel", "\n".join(build) + "\n")
    ctx.file("sdk.json", json.encode_indent({
        "schema": "robodyna.ros_sdk.v1", "distribution": pins["distribution"],
        "root": root, "prefix": pins["prefix"], "version": pins["version"],
        "libraries": records, "include_directories": pins["include_directories"],
        "definitions": pins["definitions"], "definition_resolution": pins["definition_resolution"],
        "runtime_directories": directories,
        "scope": "Declared official ROS2 C++/DDS inputs; actual node, transport and runtime gates remain required",
    }) + "\n")

local_ros = repository_rule(
    implementation = _local_ros_impl,
    attrs = {"_pins": attr.label(default = Label("//build_defs/sdk:ros_pins.json"), allow_single_file = True)},
    environ = ["ROBODYNA_ROS_ROOT"],
    local = True,
    configure = True,
)
