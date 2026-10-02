"""Build real native FMU shared objects and package their generated descriptors."""

load("@rules_cc//cc:defs.bzl", "cc_binary")
load("//build_defs/features:defs.bzl", "BASELINE_ABI_ONLY")

FmuInfo = provider(fields = ["archive", "tree", "identifier", "version"])

def _resources(_os, _inputs):
    return {"cpu": 1, "memory": 512}

def _package_impl(ctx):
    tree = ctx.actions.declare_directory(ctx.label.name + ".unpacked")
    archive = ctx.actions.declare_file(ctx.label.name + ".fmu")
    description = ctx.actions.declare_file(ctx.label.name + ".modelDescription.xml")
    receipt = ctx.actions.declare_file(ctx.label.name + ".receipt.json")
    spec = ctx.actions.declare_file(ctx.label.name + ".build.json")
    resources = []
    for file in ctx.files.resources:
        if not file.short_path.startswith(ctx.attr.resource_prefix):
            fail("FMU resource is outside its explicit source prefix: " + file.short_path)
        relative = file.short_path[len(ctx.attr.resource_prefix):]
        if not relative or relative.startswith("/"):
            fail("Invalid resource prefix")
        resources.append({"source": file.path, "destination": "resources/" + relative})
    licenses = [{"source": file.path, "destination": "documentation/licenses/" + str(i) + "-" + file.basename}
                for i, file in enumerate(ctx.files.licenses)]
    ctx.actions.write(spec, json.encode({
        "schema": "robodyna.fmu_build.v1", "identifier": ctx.attr.identifier,
        "version": ctx.attr.version, "mode": ctx.attr.mode, "guid": ctx.attr.guid,
        "library": ctx.file.library.path, "helper": ctx.executable._helper.path,
        "tree": tree.path, "archive": archive.path, "description": description.path,
        "receipt": receipt.path, "resources": resources, "licenses": licenses,
        "runtime_libraries": [{"source": file.path, "name": file.basename} for file in ctx.files.runtime_libraries],
    }))
    ctx.actions.run(
        executable = ctx.attr._package[DefaultInfo].files_to_run,
        arguments = [spec.path],
        inputs = [spec, ctx.file.library] + ctx.files.resources + ctx.files.licenses + ctx.files.runtime_libraries,
        tools = [ctx.attr._helper[DefaultInfo].files_to_run],
        outputs = [tree, archive, description, receipt],
        mnemonic = "PackageNativeFmu",
        progress_message = "Generate and package " + ctx.attr.identifier,
        resource_set = _resources,
    )
    return [DefaultInfo(files = depset([archive]), runfiles = ctx.runfiles(files = [archive])),
            FmuInfo(archive = archive, tree = tree, identifier = ctx.attr.identifier, version = ctx.attr.version),
            OutputGroupInfo(unpacked = depset([tree]), metadata = depset([description, receipt, spec]))]

_fmu_package = rule(
    implementation = _package_impl,
    attrs = {
        "identifier": attr.string(mandatory = True),
        "version": attr.string(values = ["2", "3"], mandatory = True),
        "mode": attr.string(values = ["CoSimulation", "ModelExchange"], mandatory = True),
        "guid": attr.string(mandatory = True),
        "library": attr.label(allow_single_file = True, mandatory = True),
        "resources": attr.label_list(allow_files = True),
        "resource_prefix": attr.string(),
        "runtime_libraries": attr.label_list(allow_files = True),
        "licenses": attr.label_list(allow_files = True),
        "_helper": attr.label(default = "@fmu_forge//:model_description", executable = True, cfg = "exec"),
        "_package": attr.label(default = "//src/integrations/fmi/export:package", executable = True, cfg = "exec"),
    },
)

def native_fmu(name, identifier, version, mode, guid, srcs, resources = [], resource_prefix = "", deps = [], compile_profile = None, runtime_libraries = [], extra_notices = []):
    """Use the retained exporter and native model, without a prebuilt physics library."""
    library = identifier + ".so"
    exports = "//src/integrations/fmi/export:fmi" + version + ".exports"
    cc_binary(
        name = library,
        srcs = srcs + ["@fmu_forge//:fmi" + version + "/FmuForgeExport.cpp"],
        deps = ["//src/integrations/fmi:headers"] + deps,
        local_defines = ['FMU_MODEL_IDENTIFIER=\\"' + identifier + '\\"', 'FMU_GUID=\\"' + guid + '\\"'],
        copts = ["-O3", "-fPIC"] + (["-include", "$(location " + compile_profile + ")"] if compile_profile else []),
        additional_compiler_inputs = [compile_profile] if compile_profile else [],
        linkshared = True,
        linkstatic = True,
        # FMUs own independent static mechanics state. Expose only their FMI C
        # interface: exporting C++ globals interposes host quadrature/registrar
        # objects and runs multiple destructors on the same storage at exit.
        linkopts = ["-Wl,-z,defs", "-Wl,--version-script=$(location " + exports + ")"] + (["-Wl,-rpath,$$ORIGIN"] if runtime_libraries else []),
        additional_linker_inputs = [exports],
        target_compatible_with = ["@platforms//os:linux", "@platforms//cpu:x86_64"] + BASELINE_ABI_ONLY,
        tags = ["manual", "native-fmu", "cpu-only"],
    )
    _fmu_package(
        name = name, identifier = identifier, version = version, mode = mode, guid = guid,
        library = ":" + library, resources = resources, resource_prefix = resource_prefix,
        licenses = ["//src/integrations/fmi:export_notices"] + extra_notices,
        runtime_libraries = runtime_libraries,
        target_compatible_with = ["@platforms//os:linux", "@platforms//cpu:x86_64"] + BASELINE_ABI_ONLY,
        tags = ["manual", "native-fmu", "cpu-only"],
    )
