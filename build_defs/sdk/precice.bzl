"""Explicit preCICE3.0 SDK and its unchanged packaged PETSc/MPI closure."""

load(":inspection.bzl", "file_sha256", "required_file", "shared_library", "verify_file_hashes")

def _impl(ctx):
    root = ctx.os.environ.get("ROBODYNA_PRECICE_ROOT", "")
    runtime = ctx.os.environ.get("ROBODYNA_PRECICE_RUNTIME_ROOT", "")
    if not root.startswith("/") or not runtime.startswith("/"):
        fail("Set ROBODYNA_PRECICE_ROOT and ROBODYNA_PRECICE_RUNTIME_ROOT to the admitted workspace SDK directories")
    if ctx.os.name != "linux" or ctx.os.arch not in ["amd64", "x86_64"]:
        fail("The admitted preCICE SDK is Linux x86_64")
    pins = json.decode(ctx.read(ctx.attr.pins))
    for directory, expected in [(root, pins["receipt_sha256"]), (runtime, pins["runtime_receipt_sha256"])]:
        if file_sha256(ctx, required_file(ctx, directory + "/sdk.json")) != expected:
            fail("preCICE SDK extraction receipt changed")
    verify_file_hashes(ctx, root, pins["headers"])
    platform_paths = {path["path"].lstrip("/"): path["sha256"] for path in pins["platform_libraries"].values()}
    verify_file_hashes(ctx, "/", platform_paths)
    mpi = json.decode(ctx.read(ctx.attr.mpi_receipt))
    if mpi["schema"] != "robodyna.mpi_sdk.v1":
        fail("preCICE must share the admitted MPI SDK")
    roots = {"precice": root, "runtime": runtime, "mpi": mpi["root"] + "/.."}
    records = {}
    for soname, pin in pins["libraries"].items():
        requested = roots[pin["root"]] + "/" + pin["path"]
        if pin["root"] == "mpi":
            # These libraries already have @mpi_sdk owners. Authenticate the
            # common bytes without importing another MPI implementation here.
            if file_sha256(ctx, required_file(ctx, requested)) != pin["sha256"]:
                fail("preCICE and MPI SDK identities disagree: " + soname)
            continue
        value = shared_library(ctx, requested, soname)
        if value.record["sha256"] != pin["sha256"] or value.record["needed"] != pin["needed"]:
            fail("preCICE runtime ELF identity changed: " + soname)
        records[soname] = value.record
    for path in pins["headers"]:
        ctx.symlink(root + "/" + path, path[len("usr/"):])
    ctx.symlink(required_file(ctx, root + "/usr/share/doc/libprecice3/copyright"), "LICENSE")
    # DT_RPATH is deliberate for this explicit local SDK: its transitive PETSc
    # dependencies have no own RUNPATH. Do not replace this with an undeclared
    # global LD_LIBRARY_PATH or silently choose a system preCICE/MPI install.
    runtime_dirs = [root + "/usr/lib/x86_64-linux-gnu", runtime + "/usr/lib/x86_64-linux-gnu",
                    mpi["root"] + "/lib/x86_64-linux-gnu"]
    options = ["-Wl,--disable-new-dtags", "-Wl,-rpath," + ":".join(runtime_dirs),
               "-Wl,-rpath-link," + ":".join(runtime_dirs)]
    ctx.file("BUILD.bazel", "\n".join([
        'load("@rules_cc//cc:defs.bzl", "cc_import", "cc_library")',
        'package(default_visibility = ["//visibility:public"])',
        'cc_import(name = "precice_binary", shared_library = "lib/libprecice.so.3")',
        'cc_library(name = "precice", hdrs = glob(["include/precice/*"]), includes = ["include", "include/precice"],',
        '    deps = [":precice_binary", "@mpi_sdk//:cpp", "@mpi_sdk//:fortran_runtime"],',
        '    data = [":runtime"], linkopts = ' + repr(options) + ')',
        'filegroup(name = "runtime", srcs = glob(["lib/*"]) + ["sdk.json", "LICENSE", "@mpi_sdk//:runtime"])',
        'exports_files(["sdk.json", "LICENSE"])',
    ]) + "\n")
    ctx.file("sdk.json", json.encode_indent({
        "schema": "robodyna.precice_sdk.v1", "version": pins["version"],
        "root": root, "runtime_root": runtime, "mpi_root": mpi["root"],
        "libraries": records, "platform_libraries": pins["platform_libraries"],
        "rpath_directories": runtime_dirs, "scope": pins["scope"],
    }) + "\n")

local_precice = repository_rule(
    implementation = _impl,
    attrs = {
        "pins": attr.label(default = "//build_defs/sdk:precice_pins.json", allow_single_file = True),
        "mpi_receipt": attr.label(default = "@mpi_sdk//:sdk.json", allow_single_file = True),
    },
    environ = ["ROBODYNA_PRECICE_ROOT", "ROBODYNA_PRECICE_RUNTIME_ROOT"],
    local = True,
    configure = True,
)
