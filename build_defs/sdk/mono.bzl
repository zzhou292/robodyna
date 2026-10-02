"""Pinned workspace Mono compiler/runtime, with explicit net472 references."""

load(":inspection.bzl", "file_sha256", "required_file")

def _local_mono_impl(ctx):
    if ctx.os.name != "linux" or ctx.os.arch not in ["amd64", "x86_64"]:
        fail("The declared Mono profile requires Linux x86_64")
    pins = json.decode(ctx.read(ctx.attr._pins))
    if pins["schema"] != "robodyna.mono_sdk_pins.v1":
        fail("Unsupported Mono SDK pin schema")
    root = ctx.os.environ.get(pins["environment"], "")
    if not root.startswith("/"):
        fail("Set absolute " + pins["environment"] + " to the extracted Mono SDK")
    paths = []
    for entry in pins["files"]:
        path = required_file(ctx, root + "/" + entry["path"])
        if file_sha256(ctx, path.realpath) != entry["sha256"]:
            fail("Mono SDK input differs from the pinned package: " + entry["path"])
        ctx.symlink(path.realpath, entry["path"])
        paths.append(entry["path"])
    environment = {"MONO_PATH": root + "/usr/lib/mono/4.5", "MONO_CFG_DIR": root + "/etc", "MONO_GAC_PREFIX": root + "/usr"}
    mono = root + "/usr/bin/mono-sgen"
    version = ctx.execute([mono, "--version"], environment = environment, timeout = 15)
    compiler = ctx.execute([mono, "--config", root + "/etc/mono/config", root + "/usr/lib/mono/4.5/mcs.exe", "--version"], environment = environment, timeout = 15)
    if version.return_code or compiler.return_code or pins["version"] not in version.stdout or pins["version"] not in compiler.stdout:
        fail("Declared Mono runtime/compiler version admission failed")
    ctx.file("runner.py", '''import os
import sys
from pathlib import Path
from python.runfiles import runfiles
from tools.managed.environment import mono_environment

r = runfiles.Create()
anchor = r.Rlocation(%s)
if not anchor:
    raise RuntimeError("Declared Mono runfiles are missing")
root = Path(anchor).absolute().parents[2]
tool = root / "usr/bin/mono-sgen"
arguments = sys.argv[1:]
assembly_directories = []
if arguments[:1] == ["--robodyna-assembly-dir"]:
    if len(arguments) < 3:
        raise RuntimeError("Declared managed assembly directory needs an executable")
    assembly_directories = [Path(arguments[1]).absolute()]
    arguments = arguments[2:]
environment = mono_environment(os.environ, root, assembly_directories)
os.execve(tool, [str(tool), "--config", str(root / "etc/mono/config")] + arguments, environment)
''' % repr(ctx.name + "/usr/bin/mono-sgen"))
    ctx.file("BUILD.bazel", '''load("@rules_python//python:defs.bzl", "py_binary")
package(default_visibility = ["//visibility:public"])
py_binary(name = "mono", srcs = ["runner.py"], main = "runner.py",
          deps = ["@rules_python//python/runfiles", "@//tools/managed:environment"],
          data = %s)
filegroup(name = "runtime", srcs = %s)
filegroup(name = "references", srcs = %s)
exports_files(["sdk.json"] + %s)
''' % (repr(paths), repr(paths), repr(pins["references"]), repr(paths)))
    ctx.file("sdk.json", json.encode_indent({
        "schema": "robodyna.mono_sdk.v1",
        "root": root,
        "version": pins["version"],
        "framework": pins["framework"],
        "runtime_version": version.stdout,
        "compiler_version": compiler.stdout,
        "files": pins["files"],
        "scope": "Pinned tool admission only; managed compilation and native interop require separate gates",
    }) + "\n")

local_mono = repository_rule(
    implementation = _local_mono_impl,
    attrs = {"_pins": attr.label(default = Label("//build_defs/sdk:mono_pins.json"), allow_single_file = True)},
    environ = ["ROBODYNA_MONO_ROOT"],
    local = True,
    configure = True,
)
