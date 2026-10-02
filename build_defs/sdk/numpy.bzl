"""Admit the qualified, separate NumPy wheel and C headers for CPython 3.10."""

load(":inspection.bzl", "file_sha256", "required_file")

_VERIFY = """import hashlib,json,pathlib,sys
root=pathlib.Path(sys.argv[1]); expected=sys.argv[2]
def digest(path):
    value=hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda:stream.read(1024*1024),b''):value.update(block)
    return value.hexdigest()
receipt=root/'sdk.json'
if digest(receipt)!=expected:raise SystemExit('NumPy SDK receipt pin differs')
record=json.loads(receipt.read_text())
if record.get('schema')!='robodyna.python_runtime_sdk.v1' or record['probe']['version']!='1.26.4':raise SystemExit('Unqualified NumPy SDK profile')
actual={str(path.relative_to(root)) for path in (root/'site-packages').rglob('*') if path.is_file()}
if actual!=set(record['files']):raise SystemExit('NumPy SDK file set changed')
for relative,identity in record['files'].items():
    path=root/relative
    if path.is_symlink() or path.stat().st_size!=identity['bytes'] or digest(path)!=identity['sha256']:raise SystemExit('NumPy SDK file changed: '+relative)
print(json.dumps({'files':len(actual),'version':record['probe']['version'],'python':record['probe']['python']}))
"""

def _numpy_impl(ctx):
    if ctx.os.name != "linux" or ctx.os.arch not in ["amd64", "x86_64"]:
        fail("The declared NumPy SDK profile is Linux x86_64")
    root = ctx.os.environ.get("ROBODYNA_PYTHON_RUNTIME_ROOT", "")
    if not root.startswith("/"):
        fail("Set absolute ROBODYNA_PYTHON_RUNTIME_ROOT to the qualified NumPy SDK")
    pins = json.decode(ctx.read(ctx.attr.manifest))
    receipt = required_file(ctx, root + "/sdk.json")
    if file_sha256(ctx, receipt) != pins["receipt_sha256"]:
        fail("NumPy SDK receipt differs from its reviewed pin")
    python = ctx.which("python3")
    if python == None:
        fail("NumPy SDK input verification requires the host Python utility")
    check = ctx.execute([str(python), "-I", "-B", "-c", _VERIFY, root, pins["receipt_sha256"]], timeout = 30, quiet = True)
    if check.return_code:
        fail("NumPy SDK identity verification failed: " + check.stderr)
    directory = ctx.path(root + "/site-packages")
    ctx.watch_tree(directory)
    ctx.symlink(directory, "site-packages")
    ctx.file("BUILD.bazel", """load("@rules_cc//cc:defs.bzl", "cc_library")
package(default_visibility = ["//visibility:public"])
cc_library(name = "headers", hdrs = glob(["site-packages/numpy/core/include/**/*.h"]),
           includes = ["site-packages/numpy/core/include"], deps = ["@python_sdk//:headers"])
filegroup(name = "runtime_files", srcs = glob(["site-packages/**"]))
exports_files(["site-packages/numpy/__init__.py", "sdk.json"])
""")
    ctx.file("sdk.json", json.encode_indent({
        "schema": "robodyna.local_numpy_sdk.v1",
        "root": root,
        "receipt_sha256": pins["receipt_sha256"],
        "wheel": pins["wheel"],
        "verified": json.decode(check.stdout),
        "scope": "Qualified NumPy runtime/C headers only; generated wrapper array bridges require their own native import tests.",
    }, indent = "  ") + "\n")

local_numpy_sdk = repository_rule(
    implementation = _numpy_impl,
    attrs = {"manifest": attr.label(default = Label("//build_defs/sdk:numpy_pins.json"), allow_single_file = True)},
    environ = ["ROBODYNA_PYTHON_RUNTIME_ROOT"],
    local = True,
)
