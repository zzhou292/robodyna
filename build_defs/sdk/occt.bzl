"""Admit the exact workspace-built OCCT7.9.3 CAD SDK and its full DSO closure."""

load(":inspection.bzl", "file_sha256", "required_file", "shared_library")

_VERIFY = """import hashlib,json,pathlib,sys
root=pathlib.Path(sys.argv[1]); expected=sys.argv[2]
def digest(p):
 h=hashlib.sha256()
 with p.open('rb') as f:
  for b in iter(lambda:f.read(1<<20),b''):h.update(b)
 return h.hexdigest()
receipt=root/'sdk.json'
if digest(receipt)!=expected:raise SystemExit('OCCT receipt differs from its reviewed pin')
r=json.loads(receipt.read_text())
if r.get('schema')!='robodyna.occt_built_sdk.v1' or not r.get('source_unchanged'):raise SystemExit('Unqualified OCCT source build')
if r['source']['commit']!='a016080bf6738d6aeae020badee4e888ad1540a5':raise SystemExit('Wrong OCCT source revision')
headers=[]; libraries=[]
for row in r['installed_files']:
 rel=pathlib.PurePosixPath(row['path'])
 if rel.is_absolute() or '..' in rel.parts:raise SystemExit('Invalid SDK receipt path')
 p=root/rel
 if 'symlink' in row:
  if not p.is_symlink() or str(p.readlink())!=row['symlink'] or not p.exists():raise SystemExit('OCCT symlink drift: '+str(rel))
 else:
  if not p.is_file() or p.stat().st_size!=row['bytes'] or digest(p)!=row['sha256']:raise SystemExit('OCCT file drift: '+str(rel))
  if row['path'].startswith('include/opencascade/'):headers.append(row['path'])
  if row['path'].startswith('lib/libTK') and row['path'].endswith('.so.7.9.3'):libraries.append(row)
print(json.dumps({'headers':headers,'libraries':libraries,'source':r['source'],'cache':r['expected_cache']}))
"""

def _occt_impl(ctx):
    if ctx.os.name != "linux" or ctx.os.arch not in ["amd64", "x86_64"]:
        fail("The admitted OCCT profile is Linux x86_64")
    root = ctx.os.environ.get("ROBODYNA_OCCT_ROOT", "")
    if not root.startswith("/"):
        fail("Set absolute ROBODYNA_OCCT_ROOT to the reviewed OCCT7.9.3 source installation")
    pins = json.decode(ctx.read(ctx.attr.pins))
    receipt = required_file(ctx, root + "/sdk.json")
    python = ctx.which("python3")
    if python == None:
        fail("OCCT SDK identity inspection requires Python3")
    probe = ctx.execute([str(python), "-I", "-c", _VERIFY, root, pins["sdk_receipt_sha256"]], quiet = True, timeout = 120)
    if probe.return_code:
        fail("OCCT SDK admission failed: " + probe.stderr)
    info = json.decode(probe.stdout)
    if len(info["libraries"]) != 40:
        fail("The retained33 CAD toolkits must have their qualified40-library closure")
    for directory in ["include/opencascade", "lib", "share/opencascade/resources", "share/doc/opencascade"]:
        ctx.watch_tree(ctx.path(root + "/" + directory))
    ctx.symlink(ctx.path(root + "/include/opencascade"), "include")
    ctx.symlink(ctx.path(root + "/share/opencascade/resources"), "resources")
    ctx.symlink(ctx.path(root + "/share/doc/opencascade"), "licenses")
    build = [
        'load("@rules_cc//cc:defs.bzl", "cc_import", "cc_library")',
        'package(default_visibility = ["//visibility:public"])',
        'cc_library(name = "headers", hdrs = glob(["include/**"]), includes = ["include"])',
        'filegroup(name = "runtime", srcs = glob(["lib/**", "resources/**", "licenses/**"]))',
    ]
    libraries = {}
    for row in info["libraries"]:
        toolkit = row["path"][len("lib/lib"):-len(".so.7.9.3")]
        soname = "lib" + toolkit + ".so.7.9"
        imported = shared_library(ctx, root + "/" + row["path"], soname)
        if imported.record["sha256"] != row["sha256"]:
            fail("OCCT library changed during SDK admission")
        libraries[toolkit] = imported.record
        build.append('cc_import(name = %r, shared_library = %r)' % (toolkit + "_binary", imported.output))
    platform = ["libstdc++.so.6", "libm.so.6", "libgcc_s.so.1", "libc.so.6", "libdl.so.2", "libpthread.so.0", "librt.so.1", "ld-linux-x86-64.so.2"]
    for toolkit, row in libraries.items():
        deps = [":" + toolkit + "_binary"]
        for needed in row["needed"]:
            if needed.startswith("libTK") and needed.endswith(".so.7.9"):
                owner = needed[3:-len(".so.7.9")]
                if owner not in libraries:
                    fail("Missing OCCT shared-library owner: " + needed)
                deps.append(":" + owner)
            elif needed not in platform:
                fail("Unadmitted OCCT dependency: " + needed)
        build.append('cc_library(name = %r, deps = %r)' % (toolkit, deps))
    build.extend([
        'cc_library(name = "sdk", deps = %r, data = [":runtime"], linkopts = [%r])' %
        ([":headers"] + [":" + name for name in libraries], "-Wl,-rpath," + root + "/lib"),
        'exports_files(["sdk.json", "resources/StdResource/Plugin"])',
    ])
    ctx.file("BUILD.bazel", "\n".join(build) + "\n")
    ctx.file("sdk.json", json.encode_indent({
        "schema": "robodyna.occt_sdk_admission.v1", "root": root,
        "source": info["source"], "cache": info["cache"],
        "receipt_sha256": file_sha256(ctx, receipt), "libraries": libraries,
        "headers": len(info["headers"]), "runtime_resource_root": root + "/share/opencascade/resources",
        "scope": "Pinned workspace-built CAD SDK; native consumer/runtime gates remain separate",
    }) + "\n")

local_occt = repository_rule(
    implementation = _occt_impl,
    attrs = {"pins": attr.label(default = Label("//build_defs/sdk:occt_pins.json"), allow_single_file = True)},
    environ = ["ROBODYNA_OCCT_ROOT"], local = True, configure = True,
)
