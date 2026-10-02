"""Admit pinned, separately source-built URDF libraries with real dependencies."""

load(":inspection.bzl", "file_sha256", "required_file", "shared_library", "verify_file_hashes")

_ENV = {
    "tinyxml2": "ROBODYNA_TINYXML2_ROOT",
    "console_bridge": "ROBODYNA_CONSOLE_BRIDGE_ROOT",
    "urdfdom_headers": "ROBODYNA_URDF_HEADERS_ROOT",
    "urdfdom": "ROBODYNA_URDF_ROOT",
}
_SONAMES = {
    "libtinyxml2.so.11": "tinyxml2",
    "libconsole_bridge.so.1.0": "console_bridge",
    "liburdfdom_model.so.4.0": "urdfdom",
    "liburdfdom_model_state.so.4.0": "urdfdom",
    "liburdfdom_sensor.so.4.0": "urdfdom",
    "liburdfdom_world.so.4.0": "urdfdom",
}

def _urdf_impl(ctx):
    if ctx.os.name != "linux" or ctx.os.arch not in ["amd64", "x86_64"]:
        fail("URDF SDK profile requires Linux x86_64")
    pins = json.decode(ctx.read(ctx.attr.pins))
    roots = {}
    libraries = {}
    sources = {}
    for name, env in _ENV.items():
        root = ctx.os.environ.get(env, "")
        if not root.startswith("/"):
            fail("Set " + env + " to the pinned source-built SDK installation")
        roots[name] = root
        receipt = required_file(ctx, root + "/sdk.json")
        if file_sha256(ctx, receipt) != pins["components"][name]["receipt_sha256"]:
            fail("URDF SDK receipt changed: " + name)
        info = json.decode(ctx.read(receipt))
        if info.get("schema") != "robodyna.cmake_built_sdk.v1" or info.get("source_unchanged") != True:
            fail("URDF component has no completed source-build receipt")
        if info["source"]["commit"] != pins["components"][name]["commit"]:
            fail("URDF component source revision changed")
        files = {}
        for row in info["installed_files"]:
            path = row["path"]
            if path.startswith("/") or ".." in path.split("/"):
                fail("Unsafe installed SDK path")
            if "sha256" in row:
                files[path] = row["sha256"]
        verify_file_hashes(ctx, root, files)
        sentinel = {"tinyxml2": "tinyxml2.h", "console_bridge": "console_bridge/console.h", "urdfdom_headers": "urdf_model/model.h", "urdfdom": "urdf_parser/urdf_parser.h"}[name]
        required_file(ctx, root + "/include/" + sentinel)
        ctx.watch_tree(ctx.path(root + "/include"))
        ctx.symlink(ctx.path(root + "/include"), "include/" + name)
        sources[name] = info["source"]
        for soname, owner in _SONAMES.items():
            if owner != name:
                continue
            imported = shared_library(ctx, root + "/lib/" + soname, soname)
            if imported.record["sha256"] not in files.values():
                fail("URDF DSO is absent from its exact source-build receipt: " + soname)
            libraries[soname] = imported
    for source, name in ctx.attr.licenses.items():
        ctx.symlink(ctx.path(source), "licenses/" + name)
    build = [
        'load("@rules_cc//cc:defs.bzl", "cc_import", "cc_library")',
        'package(default_visibility = ["//visibility:public"])',
        'cc_library(name="headers", hdrs=glob(["include/**/*.h", "include/**/*.hpp"], allow_empty=True), includes=%r)' % ["include/" + name for name in _ENV],
        'filegroup(name="runtime", srcs=glob(["lib/**", "licenses/**"]) + ["sdk.json"])',
    ]
    targets = {soname: soname.replace(".", "_") for soname in libraries}
    system = ["libstdc++.so.6", "libm.so.6", "libgcc_s.so.1", "libc.so.6", "libdl.so.2", "libpthread.so.0", "ld-linux-x86-64.so.2"]
    for soname, imported in libraries.items():
        name = targets[soname]
        build.append('cc_import(name=%r, shared_library=%r)' % (name + "_binary", imported.output))
        deps = [":" + name + "_binary"]
        for needed in imported.record["needed"]:
            if needed in targets:
                deps.append(":" + targets[needed])
            elif needed not in system:
                fail("URDF SDK has an unowned DSO dependency: " + needed)
        build.append('cc_library(name=%r, deps=%r)' % (name, deps))
    build.append('cc_library(name="urdf", deps=%r, data=[":runtime"], linkopts=%r)' % (
        [":headers"] + [":" + name for name in targets.values()],
        ["-Wl,-rpath," + root + "/lib" for name, root in roots.items() if name != "urdfdom_headers"],
    ))
    build.append('exports_files(["sdk.json"])')
    ctx.file("BUILD.bazel", "\n".join(build) + "\n")
    ctx.file("sdk.json", json.encode_indent({
        "schema": "robodyna.urdf_sdk_admission.v1", "roots": roots, "sources": sources,
        "libraries": {name: lib.record for name, lib in libraries.items()},
        "scope": "Source and ELF admission only; native parser/runtime tests are separate",
    }) + "\n")

local_urdf = repository_rule(
    implementation = _urdf_impl,
    attrs = {
        "pins": attr.label(default = Label("//build_defs/sdk:urdf_pins.json"), allow_single_file = True),
        "licenses": attr.label_keyed_string_dict(default = {
            Label("//build_defs/sdk/urdf_licenses:tinyxml2.txt"): "tinyxml2.txt",
            Label("//build_defs/sdk/urdf_licenses:console_bridge.txt"): "console_bridge.txt",
            Label("//build_defs/sdk/urdf_licenses:urdfdom_headers.txt"): "urdfdom_headers.txt",
            Label("//build_defs/sdk/urdf_licenses:urdfdom.txt"): "urdfdom.txt",
        }, allow_files = True),
    },
    environ = _ENV.values(),
    local = True,
    configure = True,
)
