"""Explicit local VSG development SDK used by the native rendering frontend.

Only external rendering dependencies are imported. Chrono implementation libraries
are forbidden; Robodyna compiles those from its absorbed first-party sources.
"""

def _required_file(ctx, value):
    path = ctx.path(value)
    if not path.exists or path.is_dir:
        fail("Missing declared VSG SDK file: " + value)
    ctx.watch(path)
    ctx.watch(path.realpath)
    return path

def _tool(ctx, name):
    path = ctx.which(name)
    if path == None:
        fail("VSG SDK inspection needs host tool " + name)
    return path

def _run(ctx, args):
    result = ctx.execute(args, quiet = True)
    if result.return_code != 0:
        fail("VSG SDK inspection failed: %s\n%s" % (args, result.stderr))
    return result.stdout

def _words(value):
    # Starlark split requires an explicit separator (unlike Python split()).
    normalized = value.replace("\t", " ").replace("\r", " ").replace("\n", " ")
    return [word for word in normalized.split(" ") if word]

def _macro(ctx, path, key):
    for line in ctx.read(_required_file(ctx, path)).splitlines():
        words = _words(line)
        if len(words) >= 3 and words[0] == "#define" and words[1] == key:
            return words[2].strip('"')
    fail("Missing SDK version macro " + key)

def _library(ctx, requested, expected_soname, readelf, sha256):
    path = _required_file(ctx, requested)
    dynamic = _run(ctx, [readelf, "-d", str(path.realpath)])
    needed = []
    soname = None
    runpaths = []
    for line in dynamic.splitlines():
        if "(NEEDED)" in line:
            needed.append(line.split("[", 1)[1].split("]", 1)[0])
        elif "(SONAME)" in line:
            soname = line.split("[", 1)[1].split("]", 1)[0]
        elif "(RPATH)" in line or "(RUNPATH)" in line:
            runpaths.append(line.split("[", 1)[1].split("]", 1)[0])
    if soname != expected_soname:
        fail("SDK SONAME differs: expected %s, got %s" % (expected_soname, soname))
    if "Chrono" in soname or any(["Chrono" in dependency for dependency in needed]):
        fail("External VSG SDK must not import any old Chrono implementation library")
    digest = _words(_run(ctx, [sha256, str(path.realpath)]))[0]
    output = "lib/" + soname
    ctx.symlink(path.realpath, output)
    return struct(output = output, record = {
        "requested": requested,
        "resolved": str(path.realpath),
        "sha256": digest,
        "soname": soname,
        "needed": needed,
        "embedded_runpaths": runpaths,
    })

def _local_vsg_impl(ctx):
    if ctx.os.name != "linux" or ctx.os.arch not in ["amd64", "x86_64"]:
        fail("Initial VSG SDK profile is Linux x86_64; other platforms need explicit qualification")
    root = ctx.os.environ.get("ROBODYNA_VSG_ROOT", "")
    system = ctx.os.environ.get("ROBODYNA_VSG_SYSROOT", "")
    if not root.startswith("/") or not system.startswith("/"):
        fail("Set absolute ROBODYNA_VSG_ROOT and ROBODYNA_VSG_SYSROOT for the explicit rendering SDK")
    readelf = _tool(ctx, "readelf")
    sha256 = _tool(ctx, "sha256sum")
    versions = {
        "vsg": _macro(ctx, root + "/include/vsg/core/Version.h", "VSG_VERSION_STRING"),
        "vsgXchange": _macro(ctx, root + "/include/vsgXchange/Version.h", "VSGXCHANGE_VERSION_STRING"),
        "vulkan_header": _macro(ctx, system + "/include/vulkan/vulkan_core.h", "VK_HEADER_VERSION"),
    }
    version_config = _required_file(ctx, root + "/lib/cmake/vsgImGui/vsgImGuiConfigVersion.cmake")
    versions["vsgImGui"] = ""
    for line in ctx.read(version_config).splitlines():
        if line.startswith('set(PACKAGE_VERSION "'):
            versions["vsgImGui"] = line.split('"')[1]
            break
    if versions != {"vsg": "1.1.15", "vsgXchange": "1.1.12", "vsgImGui": "0.7.0", "vulkan_header": "204"}:
        fail("Unqualified VSG SDK version profile: " + str(versions))

    for folder in ["vsg", "vsgXchange", "vsgImGui"]:
        directory = ctx.path(root + "/include/" + folder)
        ctx.watch_tree(directory)
        ctx.symlink(directory, "include/" + folder)
    for folder in ["vulkan", "xcb"]:
        directory = ctx.path(system + "/include/" + folder)
        if not directory.is_dir:
            fail("Missing declared platform headers: " + str(directory))
        ctx.watch_tree(directory)
        ctx.symlink(directory, "include/" + folder)

    requests = {
        "vsg_binary": (root + "/lib/libvsg.so", "libvsg.so.17"),
        "xchange_binary": (root + "/lib/libvsgXchange.so", "libvsgXchange.so.3"),
        "imgui_binary": (root + "/lib/libvsgImGui.so", "libvsgImGui.so.0"),
        "glslang_binary": (root + "/lib/libglslang.so", "libglslang.so.16"),
        "shader_limits_binary": (root + "/lib/libglslang-default-resource-limits.so", "libglslang-default-resource-limits.so.16"),
        "vulkan_binary": (system + "/lib/x86_64-linux-gnu/libvulkan.so", "libvulkan.so.1"),
        "xcb_binary": (system + "/lib/x86_64-linux-gnu/libxcb.so", "libxcb.so.1"),
    }
    build = [
        'load("@rules_cc//cc:defs.bzl", "cc_import", "cc_library")',
        'package(default_visibility = ["//visibility:public"])',
    ]
    records = {}
    for name, request in requests.items():
        library = _library(ctx, request[0], request[1], readelf, sha256)
        records[name] = library.record
        build.append('cc_import(name = %s, shared_library = %s)' % (repr(name), repr(library.output)))
    build.extend([
        'cc_library(name = "sdk", hdrs = glob(["include/**/*.h", "include/**/*.hpp"], allow_empty = True), includes = ["include"],',
        '    defines = ["VSG_SHARED_LIBRARY", "VSGXCHANGE_SHARED_LIBRARY", "VSGIMGUI_SHARED_LIBRARY"],',
        '    deps = %s, linkopts = ["-pthread"])' % repr([":" + name for name in requests]),
        'filegroup(name = "runtime", srcs = glob(["lib/*"]))',
        'exports_files(["sdk.json"])',
    ])
    ctx.file("BUILD.bazel", "\n".join(build) + "\n")
    ctx.file("sdk.json", json.encode_indent({
        "schema": "robodyna.local_vsg_sdk.v1",
        "sdk_root": root,
        "platform_root": system,
        "versions": versions,
        "libraries": records,
        "host_runtime": "OS C/C++ ABI, X11 authentication dependencies and installed Vulkan ICD/driver remain platform prerequisites.",
        "qualification": "Exact declared shared SDK; native frontend build and guarded render qualification remain required.",
    }) + "\n")

local_vsg = repository_rule(
    implementation = _local_vsg_impl,
    environ = ["ROBODYNA_VSG_ROOT", "ROBODYNA_VSG_SYSROOT"],
    local = True,
    configure = True,
)
