"""Assert actual Vehicle/model compile owners, including reused SCM sources."""

load("//build_defs/chrono:dependency_closure.bzl", "NativeClosure", "native_closure_aspect")
load(":sources.bzl", "VEHICLE_GROUPS")
load("//src/vehicle/models:sources.bzl", "MODEL_GROUPS")
load("//src/vehicle/visualization:sources.bzl", "VISUALIZATION_GROUPS")

def _ownership_impl(ctx):
    prefix = "src/compatibility/chrono/"
    expected = {}
    for package, groups in [("//src/vehicle", VEHICLE_GROUPS), ("//src/vehicle/models", MODEL_GROUPS)]:
        for name, group in groups.items():
            for path in group["sources"]:
                full = prefix + path
                if full in expected:
                    fail("Vehicle source appears in multiple component groups: " + full)
                expected[full] = str(Label(package + ":" + name + "_implementation"))
    for name, group in VISUALIZATION_GROUPS.items():
        for path in group["sources"]:
            expected[prefix + path] = str(Label("//src/vehicle/visualization:" + name))
    for path in ["ChTerrain.cpp", "ChWorldFrame.cpp", "ChVehicleDataPath.cpp", "terrain/SCMTerrain.cpp"]:
        expected[prefix + "src/chrono_vehicle/" + path] = str(Label("//src/compatibility/chrono:native_scm"))
    expected[prefix + "src/chrono_vehicle/visualization/ChScmVisualizationVSG.cpp"] = str(Label("//src/compatibility/chrono:native_scm_vsg"))
    # Both renderer adapters and the Vehicle module must reuse the existing STB owner.
    for path in ["stb_image.cpp", "stb_image_write.cpp"]:
        expected[prefix + "src/chrono_thirdparty/stb/" + path] = str(Label("//src/compatibility/chrono:native_stb"))
    if ctx.attr.with_crm:
        expected[prefix + "src/chrono_vehicle/terrain/CRMTerrain.cpp"] = str(Label("//src/vehicle/crm:terrain"))
    if ctx.attr.with_opencrg:
        expected[prefix + "src/chrono_vehicle/terrain/CRGTerrain.cpp"] = str(Label("//src/vehicle/opencrg:terrain"))
    found = {}
    for target in ctx.attr.components:
        for entry in target[NativeClosure].owners.to_list():
            path, owner = entry.split("|", 1)
            if not (path.startswith(prefix + "src/chrono_vehicle/") or
                    path.startswith(prefix + "src/chrono_models/vehicle/") or
                    path.startswith(prefix + "src/chrono_thirdparty/stb/")):
                continue
            if path not in expected or expected[path] != owner:
                fail("Incorrect or duplicate Vehicle/model/STB compile owner: " + entry)
            if path in found and found[path] != owner:
                fail("Vehicle source compiled twice: " + path)
            found[path] = owner
    if found != expected:
        fail("Vehicle/model closure omits an admitted source or its reused SCM/STB implementation")
    executable = ctx.actions.declare_file(ctx.label.name + ".sh")
    ctx.actions.write(executable, "#!/bin/sh\n# Exact compile-owner assertions passed during Bazel analysis.\nexit 0\n", is_executable = True)
    return [DefaultInfo(executable = executable)]

vehicle_ownership_test = rule(
    implementation = _ownership_impl,
    attrs = {
        "components": attr.label_list(mandatory = True, aspects = [native_closure_aspect]),
        "with_crm": attr.bool(default = False),
        "with_opencrg": attr.bool(default = False),
    },
    test = True,
)
