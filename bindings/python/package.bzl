"""Package actual generated proxies/extensions with one shared native backend."""

def _single_file(target, what):
    files = target[DefaultInfo].files.to_list()
    if len(files) != 1:
        fail("Expected one declared " + what + " artifact from " + str(target.label))
    return files[0]

def _package_impl(ctx):
    if ctx.attr.numpy and not ctx.file.python_runtime_anchor:
        fail("NumPy wrappers require an explicitly declared matching runtime")
    if sorted(ctx.attr.proxies.keys()) != sorted(ctx.attr.extensions.keys()) or "core" not in ctx.attr.proxies:
        fail("Python proxies and real extensions must have matching modules including core")
    manifest = ctx.actions.declare_file(ctx.label.name + "/package.json")
    runfile_depth = len(manifest.short_path.split("/")) - 1
    outputs = []
    modules = []
    for name in ["core"] + sorted([name for name in ctx.attr.proxies if name != "core"]):
        if not name or "/" in name or "." in name:
            fail("Expected a module leaf name")
        proxy = _single_file(ctx.attr.proxies[name], "Python proxy")
        extension = _single_file(ctx.attr.extensions[name], "native extension")
        for original, destination in [(proxy, "pychrono/" + name + ".py"), (extension, "pychrono/_" + name + ".so")]:
            output = ctx.actions.declare_file(ctx.label.name + "/" + destination)
            ctx.actions.symlink(output = output, target_file = original)
            outputs.append(output)
        modules.append({"name": name, "proxy_label": str(ctx.attr.proxies[name].label), "extension_label": str(ctx.attr.extensions[name].label)})
    backends = []
    names = {}
    for target in ctx.attr.backends:
        original = _single_file(target, "shared implementation")
        if original.basename in names:
            fail("Two implementation owners have the same package basename: " + original.basename)
        names[original.basename] = True
        output = ctx.actions.declare_file(ctx.label.name + "/native/" + original.basename)
        ctx.actions.symlink(output = output, target_file = original)
        outputs.append(output)
        # Preserve the declared ELF layout: native DSOs contain $ORIGIN paths
        # into Bazel's solib tree. Loading the friendly flattened symlink would
        # change that origin and lose already-declared transitive SDK libraries.
        backends.append({"path": "../" * runfile_depth + original.short_path,
                         "package_path": "native/" + original.basename,
                         "label": str(target.label)})
    if "librobodyna_core.so" not in names:
        fail("The package must declare its one shared native_core implementation")
    for source, destination in [(ctx.file.compatibility_init, "pychrono/__init__.py"), (ctx.file.public_init, "robodyna/__init__.py")]:
        output = ctx.actions.declare_file(ctx.label.name + "/" + destination)
        ctx.actions.symlink(output = output, target_file = source)
        outputs.append(output)
    runtime_roots = []
    if ctx.file.python_runtime_anchor:
        anchor = ctx.file.python_runtime_anchor
        suffix = "/numpy/__init__.py"
        if not anchor.short_path.endswith(suffix) or not ctx.attr.runtime_data:
            fail("The declared Python runtime needs its NumPy root anchor and complete runtime files")
        depth = len(manifest.short_path.split("/")) - 1
        runtime_roots.append("../" * depth + anchor.short_path[:-len(suffix)])
    for anchor in ctx.files.additional_runtime_roots:
        if anchor.basename != ".robodyna-runtime.json":
            fail("Additional Python runtime roots require their explicit SDK anchor")
        depth = len(manifest.short_path.split("/")) - 1
        runtime_roots.append("../" * depth + anchor.short_path.rsplit("/", 1)[0])
    ctx.actions.write(manifest, json.encode_indent({
        "schema": "robodyna.python_binding_package.v1",
        "python": "CPython 3.10, using the separately declared python_sdk interpreter",
        "modules": modules,
        "backends": backends,
        "numpy": ctx.attr.numpy,
        "runtime_python_roots": runtime_roots,
        "profile": ctx.attr.profile,
        "abi_capabilities": sorted(ctx.attr.abi_capabilities),
        "scope": "Actual native extensions and shared implementation inputs. Package construction is not runtime qualification.",
    }, indent = "  ") + "\n")
    outputs.append(manifest)
    runfiles = ctx.runfiles(files = outputs)
    for target in ctx.attr.extensions.values() + ctx.attr.backends + ctx.attr.runtime_data:
        info = target[DefaultInfo]
        runfiles = runfiles.merge(ctx.runfiles(transitive_files = info.files))
        if info.default_runfiles != None:
            runfiles = runfiles.merge(info.default_runfiles)
        if info.data_runfiles != None:
            runfiles = runfiles.merge(info.data_runfiles)
    return [DefaultInfo(files = depset(outputs), runfiles = runfiles),
            OutputGroupInfo(manifest = depset([manifest]))]

python_binding_package = rule(
    implementation = _package_impl,
    attrs = {
        "proxies": attr.string_keyed_label_dict(mandatory = True),
        "extensions": attr.string_keyed_label_dict(mandatory = True),
        "backends": attr.label_list(mandatory = True),
        "numpy": attr.bool(default = False),
        "python_runtime_anchor": attr.label(allow_single_file = True),
        "runtime_data": attr.label_list(),
        "additional_runtime_roots": attr.label_list(allow_files = True),
        "profile": attr.string(default = "baseline"),
        "abi_capabilities": attr.string_list(),
        "compatibility_init": attr.label(default = Label("//bindings/python:compatibility_init.py"), allow_single_file = True),
        "public_init": attr.label(default = Label("//bindings/python:public_init.py"), allow_single_file = True),
    },
)
