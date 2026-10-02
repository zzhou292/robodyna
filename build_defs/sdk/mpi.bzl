"""Pinned OpenMPI C/C++ SDK; process launch remains explicit and bounded."""

load(":inspection.bzl", "required_file", "run_tool", "shared_library", "verify_file_hashes")

def _impl(ctx):
    root = ctx.os.environ.get("ROBODYNA_MPI_ROOT", "")
    if not root.startswith("/"):
        fail("Set ROBODYNA_MPI_ROOT to the admitted OpenMPI SDK usr directory")
    pins = json.decode(ctx.read(ctx.attr.pins))
    if ctx.os.name != "linux" or ctx.os.arch not in ["amd64", "x86_64"]:
        fail("This MPI SDK profile is Linux x86_64")
    verify_file_hashes(ctx, root, pins["headers"])
    verify_file_hashes(ctx, root, pins["runtime_files"])
    for relative, target in pins["runtime_links"].items():
        path = ctx.path(root + "/" + relative)
        ctx.watch(path)
        if run_tool(ctx, "readlink", [str(path)]).strip() != target:
            fail("MPI runtime symlink differs: " + relative)
    headers = ctx.path(root + "/lib/x86_64-linux-gnu/openmpi/include")
    ctx.watch_tree(headers)
    ctx.symlink(headers, "include")
    build = [
        'load("@rules_cc//cc:defs.bzl", "cc_import", "cc_library")',
        'package(default_visibility = ["//visibility:public"])',
    ]
    records = {}
    for name, pin in pins["libraries"].items():
        value = shared_library(ctx, root + "/" + pin["path"], pin["soname"])
        if value.record["sha256"] != pin["sha256"] or value.record["needed"] != pin["needed"]:
            fail("MPI runtime identity differs: " + name)
        records[name] = value.record
        build.append('cc_import(name = %s, shared_library = %s)' % (repr(name), repr(value.output)))
    # OpenMPI opens its component plugins dynamically. Keep the exact prefix
    # available for OPAL_PREFIX/--prefix at runtime, independent of system MPI.
    for directory in ["bin", "share/openmpi", "lib/x86_64-linux-gnu/openmpi/lib"]:
        path = ctx.path(root + "/" + directory)
        if not path.is_dir:
            fail("Missing MPI runtime directory: " + str(path))
        ctx.watch_tree(path)
        ctx.symlink(path, "prefix/" + directory)
    ctx.symlink(required_file(ctx, root + "/share/doc/libopenmpi-dev/copyright").realpath, "LICENSE")
    build += [
        'cc_library(name = "c", hdrs = glob(["include/**/*.h", "include/**/*.hpp"], allow_empty = True),',
        '    includes = ["include"], deps = [":mpi_binary", ":rte_binary", ":pal_binary"], linkopts = ["-pthread"])',
        'cc_library(name = "cpp", deps = [":c", ":cxx_binary"])',
        'cc_library(name = "fortran_runtime", deps = [":c", ":fortran_binary"])',
        'filegroup(name = "runtime", srcs = glob(["lib/*", "prefix/**"]))',
        'exports_files(["sdk.json", "LICENSE", "prefix/bin/mpirun.openmpi", "prefix/bin/orted"])',
    ]
    ctx.file("BUILD.bazel", "\n".join(build) + "\n")
    ctx.file("sdk.json", json.encode_indent({"schema": "robodyna.mpi_sdk.v1", "root": root,
        "version": pins["version"], "libraries": records,
        "scope": "One pinned C/C++ MPI ABI; launching ranks and qualifying transport remain separate"}) + "\n")

local_mpi = repository_rule(
    implementation = _impl,
    attrs = {"pins": attr.label(default = "//build_defs/sdk:mpi_pins.json", allow_single_file = True)},
    environ = ["ROBODYNA_MPI_ROOT"],
    local = True,
    configure = True,
)
