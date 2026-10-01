"""Declare an existing Linux CPython 3.10 SDK for native wrapper qualification."""

_PROBE = """import hashlib,json,pathlib,platform,sys,sysconfig
if sys.implementation.name != 'cpython' or sys.version_info[:2] != (3,10):
    raise SystemExit('This initial binding profile requires CPython 3.10')
if platform.system() != 'Linux' or platform.machine() != 'x86_64':
    raise SystemExit('This initial binding profile requires Linux x86_64')
exe=pathlib.Path(sys.executable).resolve()
include=pathlib.Path(sysconfig.get_path('include')).resolve()
triplet=sysconfig.get_config_var('MULTIARCH')
multi=pathlib.Path(sysconfig.get_config_var('INCLUDEDIR'))/triplet/'python3.10'
headers=[]
for directory,prefix in [(include,'include/python3.10'),(multi,'include/'+triplet+'/python3.10')]:
    if not directory.is_dir(): raise SystemExit('Missing Python SDK include directory: '+str(directory))
    for header in sorted(directory.rglob('*.h')):
        headers.append({'source':str(header),'destination':prefix+'/'+str(header.relative_to(directory)),
                        'sha256':hashlib.sha256(header.read_bytes()).hexdigest()})
if not (include/'Python.h').is_file() or len(headers)>512:
    raise SystemExit('Missing or unexpected Python SDK headers')
print(json.dumps({'executable':str(exe),'executable_sha256':hashlib.sha256(exe.read_bytes()).hexdigest(),
                 'version':sys.version,'soabi':sysconfig.get_config_var('SOABI'),
                 'extension_suffix':sysconfig.get_config_var('EXT_SUFFIX'),'headers':headers}))
"""

def _python_bindings_impl(ctx):
    requested = ctx.os.environ.get("ROBODYNA_PYTHON_EXECUTABLE", "")
    if not requested.startswith("/"):
        fail("Set ROBODYNA_PYTHON_EXECUTABLE to the explicit CPython 3.10 interpreter")
    probe = ctx.execute([requested, "-I", "-c", _PROBE], timeout = 20)
    if probe.return_code:
        fail("Python binding SDK admission failed: " + probe.stderr)
    info = json.decode(probe.stdout)
    ctx.symlink(info["executable"], "bin/python3.10")
    ctx.watch(info["executable"])
    for header in info["headers"]:
        ctx.watch(header["source"])
        ctx.symlink(header["source"], header["destination"])
    ctx.file("sdk.json", json.encode_indent(info) + "\n")
    ctx.file("BUILD.bazel", """load("@rules_cc//cc:defs.bzl", "cc_library")
package(default_visibility = ["//visibility:public"])
cc_library(name = "headers", hdrs = glob(["include/**/*.h"]), includes = ["include", "include/python3.10"])
exports_files(["bin/python3.10", "sdk.json"])
""")

python_bindings_sdk = repository_rule(
    implementation = _python_bindings_impl,
    environ = ["ROBODYNA_PYTHON_EXECUTABLE"],
    local = True,
    configure = True,
)
