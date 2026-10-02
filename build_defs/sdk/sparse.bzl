"""Explicit pinned MUMPS-sequential and Intel 2023 CPU sparse-solver SDKs."""

load(":inspection.bzl", "file_sha256", "required_file", "shared_library")

def _admit(ctx, component):
    if ctx.os.name != "linux" or ctx.os.arch not in ["amd64", "x86_64"]:
        fail("This sparse SDK profile is qualified only for Linux x86_64")
    document = json.decode(ctx.read(ctx.attr._pins))
    if document["schema"] != "robodyna.sparse_sdk_pins.v1":
        fail("Unsupported sparse SDK pin manifest")
    spec = document[component]
    root = ctx.os.environ.get(spec["environment"], "")
    if not root.startswith("/"):
        fail("Set absolute " + spec["environment"] + " for the explicit " + component + " SDK")
    include = ctx.path(root + "/" + spec["header_directory"])
    ctx.watch_tree(include)
    ctx.symlink(include, "include")
    headers = []
    for row in spec["headers"]:
        path = required_file(ctx, root + "/" + row["path"])
        if file_sha256(ctx, path.realpath) != row["sha256"]:
            fail("Sparse SDK header differs from the pinned package: " + row["path"])
        headers.append("include/" + row["path"][len(spec["header_directory"]) + 1:])
    build = [
        'load("@rules_cc//cc:defs.bzl", "cc_import", "cc_library")',
        'package(default_visibility = ["//visibility:public"])',
    ]
    records = {}
    runtime = []
    dependencies = []
    for row in spec["libraries"] + spec["runtime_only"]:
        library = shared_library(ctx, root + "/" + row["path"], row["soname"])
        if library.record["sha256"] != row["sha256"] or library.record["needed"] != row["needed"]:
            fail("Sparse SDK binary differs from the pinned package: " + row["path"])
        records[row["name"]] = library.record
        runtime.append(library.output)
        if row in spec["libraries"]:
            if component == "mkl":
                # MKL opens dispatch DSOs beside the loaded core (dladdr), so
                # Bazel's separate per-cc_import runtime directories are not
                # a valid vendor layout. Link the same DSO as an interface and
                # let the explicit SDK RPATH load its real SONAME beside the
                # pinned companions. All actual DSOs remain declared runfiles.
                interface = "link/" + row["name"] + ".so"
                ctx.symlink(ctx.path(root + "/" + row["path"]).realpath, interface)
                build.append('cc_import(name = %s, interface_library = %s, system_provided = True)' % (repr(row["name"]), repr(interface)))
            else:
                build.append('cc_import(name = %s, shared_library = %s)' % (repr(row["name"]), repr(library.output)))
            dependencies.append(":" + row["name"])
    # Match the vendor's dynamic-library search contract. MKL dispatches to CPU
    # implementation DSOs at run time; the supplied SDK directory is explicit
    # and pinned, not a search for an old Chrono installation. Rebuild binaries
    # when relocating this local SDK. A portable installer is a later contract.
    runtime_directory = root + "/" + spec["library_directory"]
    linkopts = spec["linkopts"] + ["-Wl,-rpath," + runtime_directory]
    if component == "mkl":
        linkopts.append("-Wl,-rpath," + root + "/lib/intel64")
    ctx.file("runtime_flags.bzl", "# Same admitted vendor runtime layout as :sdk.\nRUNTIME_LINK_FLAGS = " + repr(linkopts) + "\n")
    build.append('exports_files(["runtime_flags.bzl"])')
    build.extend([
        'cc_library(name = "sdk", hdrs = %s, includes = ["include"], deps = %s, data = [":runtime"], linkopts = %s)' % (repr(headers), repr(dependencies), repr(linkopts)),
        'filegroup(name = "runtime", srcs = %s)' % repr(runtime),
        'exports_files(["sdk.json"] + %s)' % repr(runtime),
    ])
    ctx.file("BUILD.bazel", "\n".join(build) + "\n")
    ctx.file("sdk.json", json.encode_indent({
        "schema": "robodyna.local_sparse_sdk.v1",
        "component": component,
        "version": spec["version"],
        "root": root,
        "integer_profile": "32-bit MUMPS_INT" if component == "mumps" else "MKL LP64 / Intel OpenMP",
        "libraries": records,
        "header_count": len(headers),
        "platform_runtime": spec["platform_runtime"],
        "explicit_runtime_directory": runtime_directory,
        "runtime_layout": "Vendor directory via explicit RPATH; no isolated Bazel MKL runtime symlinks" if component == "mkl" else "Declared shared imports",
        "qualification": "Pinned vendor inputs; real link/solve gates remain required.",
    }) + "\n")

def _mumps_impl(ctx):
    _admit(ctx, "mumps")

def _mkl_impl(ctx):
    _admit(ctx, "mkl")

local_mumps = repository_rule(
    implementation = _mumps_impl,
    attrs = {"_pins": attr.label(default = Label("//build_defs/sdk:sparse_pins.json"), allow_single_file = True)},
    environ = ["ROBODYNA_MUMPS_ROOT"],
    local = True,
    configure = True,
)

local_mkl = repository_rule(
    implementation = _mkl_impl,
    attrs = {"_pins": attr.label(default = Label("//build_defs/sdk:sparse_pins.json"), allow_single_file = True)},
    environ = ["ROBODYNA_ONEAPI_2023_ROOT"],
    local = True,
    configure = True,
)
