"""Admit the separate pinned plotting runtime, reusing the existing NumPy owner."""

load(":inspection.bzl", "file_sha256", "required_file")

_VERIFY = """import hashlib,json,pathlib,sys
root=pathlib.Path(sys.argv[1]);record=json.loads((root/'sdk.json').read_text())
if record.get('schema')!='robodyna.python_plot_sdk.v1' or not record['probe'].get('passed') or record['probe']['backend'].lower()!='agg':raise SystemExit('Plot runtime has no qualified real Agg probe')
actual={str(p.relative_to(root)) for p in root.rglob('*') if p.is_file() or p.is_symlink()}
if actual-{'sdk.json'}!={row['path'] for row in record['files']}:raise SystemExit('Plot SDK file set changed')
for row in record['files']:
 p=root/row['path']
 if 'symlink' in row:
  if not p.is_symlink() or str(p.readlink())!=row['symlink'] or not p.exists():raise SystemExit('Plot SDK symlink changed: '+row['path'])
 else:
  h=hashlib.sha256()
  with p.open('rb') as stream:
   for b in iter(lambda:stream.read(1024*1024),b''):h.update(b)
  if p.stat().st_size!=row['bytes'] or h.hexdigest()!=row['sha256']:raise SystemExit('Plot SDK input changed: '+row['path'])
print(json.dumps({'files':len(record['files']),'packages':{k:v['version'] for k,v in record['packages'].items()},'probe':record['probe']}))
"""

def _plot_impl(ctx):
    if ctx.os.name != "linux" or ctx.os.arch not in ["amd64", "x86_64"]:
        fail("The admitted plotting runtime is Linux x86_64")
    root = ctx.os.environ.get("ROBODYNA_PYTHON_PLOT_ROOT", "")
    if not root.startswith("/"):
        fail("Set absolute ROBODYNA_PYTHON_PLOT_ROOT to the separately qualified plotting SDK")
    pins = json.decode(ctx.read(ctx.attr.pins))
    receipt = required_file(ctx, root + "/sdk.json")
    if file_sha256(ctx, receipt) != pins["receipt_sha256"]:
        fail("Plot SDK receipt differs from the reviewed pin")
    python = ctx.which("python3")
    if python == None:
        fail("SDK identity inspection requires the host Python utility")
    probe = ctx.execute([str(python), "-I", "-B", "-c", _VERIFY, root], timeout = 60, quiet = True)
    if probe.return_code:
        fail("Plot SDK file verification failed: " + probe.stderr)
    ctx.watch_tree(ctx.path(root))
    ctx.symlink(ctx.path(root + "/site-packages"), "site-packages")
    ctx.file("BUILD.bazel", """package(default_visibility = ["//visibility:public"])
filegroup(name = "runtime_files", srcs = glob(["site-packages/**"], exclude = ["site-packages/.robodyna-runtime.json"]) + ["site-packages/.robodyna-runtime.json"])
exports_files(["site-packages/.robodyna-runtime.json", "sdk.json"])
""")
    ctx.file("sdk.json", json.encode_indent({"schema": "robodyna.local_python_plot_sdk.v1", "root": root,
                                            "receipt_sha256": pins["receipt_sha256"], "verified": json.decode(probe.stdout),
                                            "scope": "Exact plotting wheel runtime; NumPy remains separately owned; GUI demos are not qualified by Agg."}, indent = "  ") + "\n")

local_python_plot_sdk = repository_rule(
    implementation = _plot_impl,
    attrs = {"pins": attr.label(default = Label("//build_defs/sdk:python_plot_pins.json"), allow_single_file = True)},
    environ = ["ROBODYNA_PYTHON_PLOT_ROOT"], local = True,
)
