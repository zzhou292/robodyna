"""Admit pinned workspace CUDA math libraries; retain the host runtime/CCCL owner."""

load(":inspection.bzl", "file_sha256", "required_file", "shared_library")

# Exact shared ABI names from the admitted NVIDIA13.2.2 / cuDSS CUDA13 archives.
_LIBRARIES = [
    ("cublas_lt", "libcublas", "libcublasLt.so", "libcublasLt.so.13", []),
    ("cublas", "libcublas", "libcublas.so", "libcublas.so.13", ["cublas_lt"]),
    ("cusparse", "libcusparse", "libcusparse.so", "libcusparse.so.12", ["nvjitlink"]),
    ("nvjitlink", "libnvjitlink", "libnvJitLink.so", "libnvJitLink.so.13", []),
    ("cusolver", "libcusolver", "libcusolver.so", "libcusolver.so.12", ["cublas", "cusparse", "nvjitlink"]),
    ("nvrtc_builtins", "cuda_nvrtc", "libnvrtc-builtins.so", "libnvrtc-builtins.so.13.2", []),
    ("nvrtc", "cuda_nvrtc", "libnvrtc.so", "libnvrtc.so.13", ["nvrtc_builtins"]),
    ("cudss", "libcudss", "libcudss.so", "libcudss.so.0", ["cublas"]),
]

def _macro_value(text, name):
    for line in text.splitlines():
        fields = [word for word in line.replace("\t", " ").split(" ") if word]
        if len(fields) >= 3 and fields[0] == "#define" and fields[1] == name:
            return fields[2]
    fail("Missing toolkit version macro " + name)

def _check_internal_dependencies(libraries):
    owners = {soname: name for name, _, _, soname, _ in _LIBRARIES}
    edges = {name: deps for name, _, _, _, deps in _LIBRARIES}
    for name, record in libraries.items():
        reachable = {name: True}
        for _ in range(len(edges)):
            for owner in list(reachable.keys()):
                for dependency in edges[owner]:
                    reachable[dependency] = True
        for needed in record["needed"]:
            if needed in owners and owners[needed] not in reachable:
                fail("CUDA SDK is missing a declared link dependency: %s -> %s" % (name, needed))

def _cuda_math_impl(ctx):
    if ctx.os.name != "linux" or ctx.os.arch not in ["amd64", "x86_64"]:
        fail("The pinned CUDA math SDK profile is Linux x86_64 only")
    root = ctx.os.environ.get("ROBODYNA_CUDA_MATH_ROOT", "")
    toolkit = ctx.os.environ.get("CUDA_PATH", "")
    if not root.startswith("/") or not toolkit.startswith("/"):
        fail("Set absolute ROBODYNA_CUDA_MATH_ROOT and the same CUDA_PATH used by rules_cuda")
    expected = json.decode(ctx.read(ctx.attr.manifest))
    receipt_file = required_file(ctx, root + "/sdk.json")
    if file_sha256(ctx, receipt_file) != expected["sdk_receipt_sha256"]:
        fail("CUDA math SDK receipt differs from its reviewed exact pin")
    receipt = json.decode(ctx.read(receipt_file))
    if receipt.get("schema") != "robodyna.cuda_math_sdk.v1" or receipt["cuda_release"] != expected["cuda_release"]:
        fail("CUDA math SDK release/schema mismatch")
    nvcc = required_file(ctx, toolkit + "/bin/nvcc")
    version = ctx.execute([str(nvcc), "--version"], quiet = True)
    if version.return_code or ("V" + expected["nvcc_version"]) not in version.stdout:
        fail("CUDA compiler differs from the admitted math SDK profile: " + version.stdout)

    headers = {}
    for component, spec in expected["components"].items():
        actual = receipt["components"][component]
        if actual["sha256"] != spec["sha256"] or actual["version"] != spec["version"]:
            fail("CUDA math component pin mismatch: " + component)
        header_names = []
        for name, identity in actual["files"].items():
            if name.startswith("include/") and "sha256" in identity:
                path = required_file(ctx, root + "/" + actual["root"] + "/" + name)
                if file_sha256(ctx, path) != identity["sha256"]:
                    fail("CUDA SDK header differs from its authenticated archive: " + name)
                destination = name
                if destination in headers:
                    fail("Overlapping CUDA math header owner: " + destination)
                ctx.symlink(path, destination)
                headers[destination] = component
                header_names.append(destination)
            if name.startswith("LICENSE") and "sha256" in identity:
                path = required_file(ctx, root + "/" + actual["root"] + "/" + name)
                if file_sha256(ctx, path) != identity["sha256"]:
                    fail("CUDA SDK license differs from admitted archive")
                ctx.symlink(path, "licenses/" + component + "/" + name)
        actual["declared_headers"] = header_names

    cccl = toolkit + "/" + expected["cccl"]["relative_include"]
    for name, macro, value in [("thrust/version.h", "THRUST_VERSION", "300200"), ("cub/version.cuh", "CUB_VERSION", "300200")]:
        path = required_file(ctx, cccl + "/" + name)
        if _macro_value(ctx.read(path), macro) != value:
            fail("Host CCCL version differs from the admitted CUDA13.2 profile")
    ctx.symlink(ctx.path(cccl), "cccl")
    ctx.watch_tree(ctx.path(cccl))
    jitify = receipt["components"]["jitify"]
    jitify_header = required_file(ctx, root + "/" + jitify["root"] + "/jitify.hpp")
    if file_sha256(ctx, jitify_header) != jitify["files"]["jitify.hpp"]["sha256"]:
        fail("Jitify header differs from the retained NVIDIA revision")
    ctx.symlink(jitify_header, "include/jitify.hpp")
    driver_stub = required_file(ctx, toolkit + "/lib64/stubs/libcuda.so")
    # This is link-time API metadata only. Never put the stub in runtime data;
    # libcuda.so.1 must come from the workstation's installed NVIDIA driver.
    ctx.symlink(driver_stub, "driver/libcuda.ifso")

    build = [
        'load("@rules_cc//cc:defs.bzl", "cc_import", "cc_library")',
        'package(default_visibility = ["//visibility:public"])',
        'filegroup(name = "runtime_files", srcs = glob(["lib/**"]))',
        'filegroup(name = "foreign_files", srcs = glob(["include/**", "lib/**", "licenses/**", "cccl/**"]))',
        'cc_library(name = "cccl_headers", hdrs = glob(["cccl/**"]), includes = ["cccl"], deps = ["@rules_cuda//cuda:runtime"])',
        'cc_import(name = "driver", interface_library = "driver/libcuda.ifso", system_provided = True)',
    ]
    libraries = {}
    for name, component, filename, soname, dependencies in _LIBRARIES:
        admitted = receipt["components"][component]
        imported = shared_library(ctx, root + "/" + admitted["root"] + "/lib/" + filename, soname)
        real_name = imported.record["resolved"].split("/")[-1]
        if imported.record["sha256"] != admitted["files"]["lib/" + real_name]["sha256"]:
            fail("CUDA shared library differs from its pinned archive: " + filename)
        # Preserve vendor companion names for dlopen (notably NVRTC builtins).
        for alias in [filename, real_name]:
            if alias != soname:
                ctx.symlink(ctx.path(imported.output), "lib/" + alias)
        libraries[name] = imported.record
        build.append('cc_import(name = %r, shared_library = %r)' % (name + "_binary", imported.output))
        deps = [":" + name + "_binary", "@rules_cuda//cuda:runtime"] + [":" + value for value in dependencies]
        # Standard cc_import supplies Bazel's solib runpaths. Companion files are
        # declared together; no second cudart or global LD_LIBRARY_PATH is added.
        build.append('cc_library(name = %r, hdrs = %r, includes = ["include"], deps = %r, data = [":runtime_files"])' %
                     (name, admitted["declared_headers"], deps))
    _check_internal_dependencies(libraries)
    build.append('cc_library(name = "jitify", hdrs = ["include/jitify.hpp"], includes = ["include"], deps = [":driver", ":nvrtc", ":cccl_headers", "@rules_cuda//cuda:runtime"])')
    build.append('exports_files(["sdk.json"])')
    ctx.file("BUILD.bazel", "\n".join(build) + "\n")
    ctx.file("sdk.json", json.encode_indent({
        "schema": "robodyna.cuda_math_provider.v1",
        "sdk_root": root,
        "toolkit_root": toolkit,
        "sdk_receipt_sha256": expected["sdk_receipt_sha256"],
        "nvcc_version": version.stdout,
        "cccl_root": cccl,
        "libraries": libraries,
        "runtime_owner": "@rules_cuda//cuda:runtime",
        "driver_link_stub": str(driver_stub.realpath),
        "driver_runtime": "system provided; no stub admitted as a runtime library",
        "scope": "Pinned local math SDK. Native compilation/link/runtime qualification remains separate.",
    }) + "\n")

cuda_math_sdk = repository_rule(
    implementation = _cuda_math_impl,
    attrs = {"manifest": attr.label(default = Label("//build_defs/sdk:cuda_math_manifest.json"), allow_single_file = True)},
    environ = ["ROBODYNA_CUDA_MATH_ROOT", "CUDA_PATH"],
    local = True,
    configure = True,
)
