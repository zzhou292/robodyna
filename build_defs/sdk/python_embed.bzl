"""Explicit CPython embedding library, paired with the existing binding headers."""

load(":inspection.bzl", "shared_library")

_PROBE = """import json,pathlib,platform,sys,sysconfig
if sys.implementation.name!='cpython' or sys.version_info[:2]!=(3,10):
 raise SystemExit('Embedding requires the qualified CPython3.10 interpreter')
if platform.system()!='Linux' or platform.machine()!='x86_64':
 raise SystemExit('Embedding profile is Linux x86_64')
print(json.dumps({'executable':str(pathlib.Path(sys.executable).resolve()),'version':sys.version,
 'soabi':sysconfig.get_config_var('SOABI'),
 'library':str(pathlib.Path(sysconfig.get_config_var('LIBDIR'))/sysconfig.get_config_var('INSTSONAME')),
 'soname':sysconfig.get_config_var('INSTSONAME')}))
"""

def _python_embed_impl(ctx):
    requested = ctx.os.environ.get("ROBODYNA_PYTHON_EXECUTABLE", "")
    if not requested.startswith("/"):
        fail("Set ROBODYNA_PYTHON_EXECUTABLE to the same interpreter used by @python_sdk")
    result = ctx.execute([requested, "-I", "-c", _PROBE], timeout = 20)
    if result.return_code:
        fail("Python embedding SDK admission failed: " + result.stderr)
    info = json.decode(result.stdout)
    library = shared_library(ctx, info["library"], info["soname"])
    allowed = ["libm.so.6", "libc.so.6", "libexpat.so.1", "libz.so.1", "libcrypt.so.1", "libdl.so.2", "libpthread.so.0", "libutil.so.1", "ld-linux-x86-64.so.2"]
    for name in library.record["needed"]:
        if name not in allowed:
            fail("Unadmitted CPython embedding dependency: " + name)
    info["library_identity"] = library.record
    ctx.file("sdk.json", json.encode_indent(info) + "\n")
    ctx.file("BUILD.bazel", """load("@rules_cc//cc:defs.bzl", "cc_import", "cc_library")
package(default_visibility = ["//visibility:public"])
cc_import(name = "library", shared_library = %r)
cc_library(name = "embed", deps = [":library", "@python_sdk//:headers"])
filegroup(name = "runtime", srcs = [%r, "sdk.json"])
exports_files(["sdk.json"])
""" % (library.output, library.output))

local_python_embed = repository_rule(
    implementation = _python_embed_impl,
    environ = ["ROBODYNA_PYTHON_EXECUTABLE"],
    local = True,
    configure = True,
)
