"""Explicit workspace-local SWIG SDK; never downloads or installs a tool."""

def _local_swig_impl(ctx):
    root = ctx.os.environ.get("ROBODYNA_SWIG_ROOT", "")
    if not root.startswith("/"):
        fail("Set ROBODYNA_SWIG_ROOT to the absolute verified local SDK directory")
    receipt_path = ctx.path(root + "/sdk.json")
    ctx.watch(receipt_path)
    receipt = json.decode(ctx.read(receipt_path))
    executable = ctx.path(root + "/usr/bin/swig4.0")
    library = ctx.path(root + "/usr/share/swig4.0")
    if not executable.exists or not library.is_dir:
        fail("Declared SWIG executable or library directory is missing")
    checksum = ctx.execute(["sha256sum", str(executable)], timeout = 10)
    if checksum.return_code or checksum.stdout.strip().split(" ")[0] != receipt.get("executable_sha256"):
        fail("SWIG executable differs from the verified SDK receipt")
    version = ctx.execute([str(executable), "-version"], environment = {"SWIG_LIB": str(library)}, timeout = 10)
    if version.return_code or "SWIG Version 4.0.2" not in version.stdout:
        fail("This qualified wrapper profile requires the explicit SWIG 4.0.2 SDK")
    ctx.watch(executable)
    ctx.watch(library)
    ctx.symlink(executable, "bin/swig4.0")
    ctx.symlink(library, "lib")
    ctx.file("runner.py", """import os
import sys
from python.runfiles import runfiles
r = runfiles.Create()
tool = r.Rlocation(%s)
anchor = r.Rlocation(%s)
if not tool or not anchor:
    raise RuntimeError("Declared SWIG runfiles are missing")
env = dict(os.environ, SWIG_LIB=os.path.dirname(anchor))
os.execve(tool, [tool] + sys.argv[1:], env)
""" % (repr(ctx.name + "/bin/swig4.0"), repr(ctx.name + "/lib/swig.swg")))
    ctx.file("sdk.json", json.encode_indent({
        "root": root,
        "executable_sha256": receipt["executable_sha256"],
        "package_sha256": receipt["package_sha256"],
        "version_output": version.stdout,
        "scope": "Declared local Linux SWIG 4.0.2 tool and its standard interface library",
    }) + "\n")
    ctx.file("BUILD.bazel", """load("@rules_python//python:defs.bzl", "py_binary")
package(default_visibility = ["//visibility:public"])
py_binary(name = "swig", srcs = ["runner.py"], main = "runner.py",
          deps = ["@rules_python//python/runfiles"],
          data = ["bin/swig4.0"] + glob(["lib/**"], allow_empty = False))
exports_files(["sdk.json"])
""")

local_swig = repository_rule(
    implementation = _local_swig_impl,
    environ = ["ROBODYNA_SWIG_ROOT"],
    local = True,
    configure = True,
)
