"""Admit source-built PythonOCC against the same exact native OCCT SDK."""

load(":inspection.bzl", "file_sha256", "required_file")

_VERIFY = """import hashlib,json,pathlib,sys,subprocess,re,importlib.util
root=pathlib.Path(sys.argv[1]);record=json.loads((root/'sdk.json').read_text())
if record.get('schema')!='robodyna.cmake_built_sdk.v1' or not record.get('source_unchanged'):raise SystemExit('PythonOCC source build is unqualified')
if record['source']['commit']!='779838f352694711d60c1567ed3f6e06f3826aa2':raise SystemExit('Wrong PythonOCC source revision')
selection=json.loads((root/'selection.json').read_text())
if selection['modules']!=json.loads(sys.argv[4]) or selection['selected_modules']!=len(selection['modules']):raise SystemExit('PythonOCC module selection differs')
cache=record['expected_cache']
if cache['CMAKE_BUILD_WITH_INSTALL_RPATH']!='ON':raise SystemExit('Selected artifacts lack their final runtime layout')
if cache['PYTHONOCC_MESHDS_NUMPY']!='ON' or cache['Python3_NumPy_INCLUDE_DIR']!=sys.argv[3]+'/site-packages/numpy/core/include':raise SystemExit('PythonOCC used an undeclared NumPy C API')
if cache['OpenCASCADE_DIR']!=sys.argv[2]+'/lib/cmake/opencascade':raise SystemExit('PythonOCC used another OCCT installation')
if cache['Python3_EXECUTABLE']!='/usr/bin/python3.10':raise SystemExit('PythonOCC used another Python SDK')
if any(cache[name]!='OFF' for name in ['PYTHONOCC_WRAP_VISU','PYTHONOCC_WRAP_OCAF','PYTHONOCC_WRAP_DATAEXCHANGE']):raise SystemExit('Unexpected PythonOCC module profile')
actual={str(p.relative_to(root)) for p in root.rglob('*') if p.is_file() or p.is_symlink()}
if actual-{'sdk.json'}!={row['path'] for row in record['installed_files']}:raise SystemExit('PythonOCC installed file set changed')
for row in record['installed_files']:
 p=root/row['path']
 if 'symlink' in row:
  if not p.is_symlink() or str(p.readlink())!=row['symlink'] or not p.exists():raise SystemExit('PythonOCC symlink changed')
 else:
  h=hashlib.sha256()
  with p.open('rb') as stream:
   for b in iter(lambda:stream.read(1024*1024),b''):h.update(b)
  if p.stat().st_size!=row['bytes'] or h.hexdigest()!=row['sha256']:raise SystemExit('PythonOCC file changed: '+row['path'])
spec=importlib.util.spec_from_file_location('occt_library_order',sys.argv[5]);audit=importlib.util.module_from_spec(spec);spec.loader.exec_module(audit)
occt=json.loads(pathlib.Path(sys.argv[6]).read_text());libraries={row['soname']:row for row in occt['libraries'].values()}
roots=set()
for name in selection['modules']:
 elf=root/'site-packages/OCC/Core'/('_'+name+'.so')
 result=subprocess.run([sys.argv[7],'-d',str(elf)],capture_output=True,text=True,check=True)
 roots.update(line.split('[',1)[1].split(']',1)[0] for line in result.stdout.splitlines() if '(NEEDED)' in line and '[libTK' in line)
order=audit.dependency_order(libraries,roots)
preloads=[{'path':libraries[name]['resolved'],'soname':name,'sha256':libraries[name]['sha256']} for name in order]
print(json.dumps({'files':len(record['installed_files']),'source':record['source'],'cache':cache,'selected_modules':selection['modules'],'native_libraries':preloads}))
"""

def _pythonocc_impl(ctx):
    if ctx.os.name != "linux" or ctx.os.arch not in ["amd64", "x86_64"]:
        fail("The admitted PythonOCC profile is Linux x86_64")
    root = ctx.os.environ.get("ROBODYNA_PYTHONOCC_ROOT", "")
    if not root.startswith("/"):
        fail("Set absolute ROBODYNA_PYTHONOCC_ROOT to the reviewed separate source installation")
    pins = json.decode(ctx.read(ctx.attr.pins))
    receipt = required_file(ctx, root + "/sdk.json")
    if file_sha256(ctx, receipt) != pins["receipt_sha256"]:
        fail("PythonOCC source-build receipt differs from the reviewed pin")
    generator = required_file(ctx, pins["swig_executable"])
    if file_sha256(ctx, generator) != pins["swig_executable_sha256"]:
        fail("The separate PythonOCC SWIG generator changed")
    generator_receipt = required_file(ctx, str(generator.dirname.dirname) + "/sdk.json")
    if file_sha256(ctx, generator_receipt) != pins["swig_receipt_sha256"]:
        fail("The separate source-built SWIG receipt changed")
    occt = json.decode(ctx.read(ctx.attr.occt_receipt))
    if occt["receipt_sha256"] != pins["occt_receipt_sha256"]:
        fail("PythonOCC and native CAD must use the same qualified OCCT owner")
    numpy = json.decode(ctx.read(ctx.attr.numpy_receipt))
    if numpy["receipt_sha256"] != pins["numpy_receipt_sha256"]:
        fail("PythonOCC must reuse the same qualified NumPy SDK")
    python = ctx.which("python3")
    if python == None:
        fail("PythonOCC SDK identity inspection requires the host Python utility")
    readelf = ctx.which("readelf")
    if readelf == None:
        fail("PythonOCC DSO closure inspection requires the platform readelf tool")
    result = ctx.execute([str(python), "-I", "-B", "-c", _VERIFY, root, occt["root"], numpy["root"], json.encode(pins["modules"]),
                          str(ctx.path(ctx.attr.library_audit)), str(ctx.path(ctx.attr.occt_receipt)), str(readelf)], quiet = True, timeout = 90)
    if result.return_code:
        fail("PythonOCC SDK verification failed: " + result.stderr)
    info = json.decode(result.stdout)
    if info["cache"]["SWIG_EXECUTABLE"] != pins["swig_executable"]:
        fail("PythonOCC used a different wrapper runtime generator")
    ctx.watch_tree(ctx.path(root))
    ctx.symlink(ctx.path(root + "/site-packages/OCC"), "runtime/OCC")
    ctx.file("runtime/.robodyna-runtime.json", json.encode_indent({
        "schema": "robodyna.python_runtime_root.v1", "kind": "pythonocc",
        "source_commit": info["source"]["commit"], "occt_root": occt["root"],
        "native_libraries": info["native_libraries"],
        "occt_receipt_sha256": occt["receipt_sha256"], "source_build_receipt_sha256": pins["receipt_sha256"],
        "scope": "Complete57-module import closure of the three retained CAD mains; other127model and optional wrapper families are not claimed. Cross-shape runtime gates remain separate.",
    }, indent = "  ") + "\n")
    ctx.file("sdk.json", json.encode_indent({"schema": "robodyna.local_pythonocc_sdk.v1", "root": root,
                                            "verified": info, "occt_receipt_sha256": occt["receipt_sha256"],
                                            "swig_runtime": 4,
                                            "version_note": "Upstream7.9.3 PkgBase retains VERSION7.9.0; exact source/build identities are authoritative."}, indent = "  ") + "\n")
    ctx.file("BUILD.bazel", """package(default_visibility = ["//visibility:public"])
filegroup(name = "runtime_files", srcs = glob(["runtime/OCC/**"]) + ["runtime/.robodyna-runtime.json"])
exports_files(["sdk.json", "runtime/.robodyna-runtime.json"])
""")

local_pythonocc_sdk = repository_rule(
    implementation = _pythonocc_impl,
    attrs = {
        "pins": attr.label(default = Label("//build_defs/sdk:pythonocc_pins.json"), allow_single_file = True),
        "occt_receipt": attr.label(default = Label("@occt_sdk//:sdk.json"), allow_single_file = True),
        "numpy_receipt": attr.label(default = Label("@numpy_sdk//:sdk.json"), allow_single_file = True),
        "library_audit": attr.label(default = Label("//tools/dependencies/pythonocc:libraries.py"), allow_single_file = True),
    },
    environ = ["ROBODYNA_PYTHONOCC_ROOT"], local = True,
)
