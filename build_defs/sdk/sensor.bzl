"""Pinned local Sensor windowing SDK and standalone GLSL compiler admission."""

load(":inspection.bzl", "file_sha256", "required_file", "run_tool", "shared_library")

def _local_sensor_impl(ctx):
    if ctx.os.name != "linux" or ctx.os.arch not in ["amd64", "x86_64"]:
        fail("The initial Sensor SDK profile is Linux x86_64")
    root = ctx.os.environ.get("ROBODYNA_SENSOR_ROOT", "")
    if not root.startswith("/"):
        fail("Set an absolute ROBODYNA_SENSOR_ROOT for the prepared Sensor SDK")
    record = json.decode(ctx.read(required_file(ctx, root + "/sdk.json")))
    pins = json.decode(ctx.read(ctx.attr._pins))
    if record.get("schema") != "robodyna.sensor_sdk.v1" or record.get("packages") != pins:
        fail("Sensor SDK package receipt differs from the reviewed graphics pins")
    tool_record = record.get("tool", {})
    if tool_record.get("source_commit") != "b5782e52ee2f7b3e40bb9c80d15b47016e008bc9" or tool_record.get("source_archive_sha256") != "6c242598c332050bf3a3619d299594ca2909426935f2c86c69c7356b049347c7":
        fail("Sensor shader compiler does not come from pinned glslang 16.1.0")
    tool = required_file(ctx, root + "/bin/glslangValidator")
    if file_sha256(ctx, tool) != tool_record.get("sha256"):
        fail("Sensor shader compiler bytes differ from the build receipt")
    version = ctx.execute([str(tool), "--version"], quiet = True)
    if version.return_code or version.stdout != tool_record.get("version_output") or "16.1.0" not in version.stdout:
        fail("Sensor shader compiler version differs from the qualified source profile")
    dynamic = run_tool(ctx, "readelf", ["-d", str(tool)])
    for forbidden in ["libglslang", "libSPIRV", "libChrono"]:
        if forbidden in dynamic:
            fail("The standalone shader compiler has an undeclared shared dependency: " + forbidden)
    ctx.symlink(tool, "bin/glslangValidator")

    for directory in ["GL", "GLFW", "KHR"]:
        path = ctx.path(root + "/graphics/usr/include/" + directory)
        if not path.is_dir:
            fail("Missing Sensor graphics header directory: " + str(path))
        ctx.watch_tree(path)
        ctx.symlink(path, "include/" + directory)
    for relative, expected in record["files"].items():
        if relative.startswith("graphics/usr/include/") and "sha256" in expected:
            if file_sha256(ctx, required_file(ctx, root + "/" + relative)) != expected["sha256"]:
                fail("Sensor SDK header changed after provisioning: " + relative)

    requested = {
        "glew": ("libGLEW.so", "libGLEW.so.2.2"),
        "glfw": ("libglfw.so", "libglfw.so.3"),
        "gl": ("libGL.so", "libGL.so.1"),
        "glx": ("libGLX.so", "libGLX.so.0"),
        "dispatch": ("libGLdispatch.so.0", "libGLdispatch.so.0"),
        "glu": ("libGLU.so", "libGLU.so.1"),
        "opengl": ("libOpenGL.so", "libOpenGL.so.0"),
    }
    build = [
        'load("@rules_cc//cc:defs.bzl", "cc_import", "cc_library")',
        'package(default_visibility = ["//visibility:public"])',
    ]
    libraries = {}
    for name, pair in requested.items():
        path = root + "/graphics/usr/lib/x86_64-linux-gnu/" + pair[0]
        library = shared_library(ctx, path, pair[1])
        resolved = str(ctx.path(path).realpath)
        if not resolved.startswith(root + "/"):
            fail("Sensor SDK library resolves outside its declared prefix: " + resolved)
        relative = resolved[len(root) + 1:]
        if record["files"].get(relative, {}).get("sha256") != library.record["sha256"]:
            fail("Sensor SDK library changed after provisioning: " + relative)
        libraries[name] = library.record
        build.append('cc_import(name = %s, shared_library = %s)' % (repr(name), repr(library.output)))
    build.extend([
        'cc_library(name = "windowing", hdrs = glob(["include/**/*.h"]), includes = ["include"],',
        '    deps = %s)' % repr([":" + name for name in requested]),
        'filegroup(name = "runtime", srcs = glob(["lib/*"]))',
        'exports_files(["bin/glslangValidator", "sdk.json"])',
    ])
    ctx.file("BUILD.bazel", "\n".join(build) + "\n")
    ctx.file("sdk.json", json.encode_indent({
        "schema": "robodyna.local_sensor_sdk.v1",
        "sdk_root": root,
        "shader_compiler": tool_record,
        "libraries": libraries,
        "host_runtime": "Host C/C++ ABI, X11 and display-driver dispatch remain platform prerequisites. No driver is supplied or replaced.",
        "qualification": "Declared native SDK input; module compilation and actual GPU rendering are separate gates.",
    }, indent = "  ") + "\n")

local_sensor_sdk = repository_rule(
    implementation = _local_sensor_impl,
    local = True,
    environ = ["ROBODYNA_SENSOR_ROOT"],
    attrs = {"_pins": attr.label(default = Label("//build_defs/sdk:sensor_pins.json"), allow_single_file = True)},
)
