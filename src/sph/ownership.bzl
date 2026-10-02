"""Check actual FSI/SPH compile owners and private/public backend definition scope."""

load("@rules_cc//cc/common:cc_info.bzl", "CcInfo")
load("//build_defs/chrono:dependency_closure.bzl", "NativeClosure", "native_closure_aspect")
load("//src/coupling/fsi:sources.bzl", "FSI_SOURCES")
load(":sources.bzl", "SPH_CUDA_SOURCES", "SPH_VISUAL_SOURCES")

def _ownership_impl(ctx):
    prefix = "src/compatibility/chrono/"
    expected = {}
    for target, sources in [
        ("//src/coupling/fsi:fsi", FSI_SOURCES),
        ("//src/sph:sph", SPH_CUDA_SOURCES),
        ("//src/sph/visualization:vsg", SPH_VISUAL_SOURCES),
        ("//src/compatibility/chrono:native_stb", ["src/chrono_thirdparty/stb/stb_image.cpp", "src/chrono_thirdparty/stb/stb_image_write.cpp"]),
    ]:
        for source in sources:
            if prefix + source in expected:
                fail("FSI/SPH source appears in two planned owners: " + source)
            expected[prefix + source] = str(Label(target))
    found = {}
    for entry in ctx.attr.implementation[NativeClosure].owners.to_list():
        source, owner = entry.split("|", 1)
        if not (source.startswith(prefix + "src/chrono_fsi/") or
                source.startswith(prefix + "src/chrono_thirdparty/stb/")):
            continue
        if source not in expected or expected[source] != owner:
            fail("Unadmitted or duplicate FSI/SPH/STB compile owner: " + entry)
        found[source] = owner
    if found != expected:
        fail("FSI/SPH closure omits an admitted source")
    for component in [ctx.attr.solver, ctx.attr.implementation]:
        definitions = component[CcInfo].compilation_context.defines.to_list()
        if "CHRONO_USE_CUDA" not in definitions:
            fail("Public SPH headers require their actual CUDA runtime definition")
        for definition in definitions:
            if definition.startswith("THRUST_DEVICE_SYSTEM") or definition.startswith("THRUST_HOST_SYSTEM") or definition == "CH_API_COMPILE_FSI":
                fail("SPH implementation definitions leaked to consumer targets: " + definition)
    executable = ctx.actions.declare_file(ctx.label.name + ".sh")
    ctx.actions.write(executable, "#!/bin/sh\n# Source owner and definition-scope checks passed during Bazel analysis.\nexit 0\n", is_executable = True)
    return [DefaultInfo(executable = executable)]

sph_ownership_test = rule(
    implementation = _ownership_impl,
    attrs = {
        "implementation": attr.label(mandatory = True, aspects = [native_closure_aspect]),
        "solver": attr.label(mandatory = True, providers = [CcInfo]),
    },
    test = True,
)
