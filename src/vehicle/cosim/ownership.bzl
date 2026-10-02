"""Inspect the real compile graph, including conditional terrain ownership."""

load("//build_defs/chrono:dependency_closure.bzl", "NativeClosure", "native_closure_aspect")
load(":sources.bzl", "COSIM_GROUPS")

def _impl(ctx):
    prefix = "src/compatibility/chrono/"
    enabled = ["base", "mbs", "robot", "rigid_tires", "flexible_tires", "cpu_terrain"]
    if ctx.attr.with_multicore:
        enabled.append("omp_terrain")
    if ctx.attr.with_sph:
        enabled.append("sph_terrain")
    if ctx.attr.with_dem:
        enabled.append("dem_terrain")
    expected = {prefix + p: str(Label("//src/vehicle/cosim:" + group)) for group in enabled for p in COSIM_GROUPS[group]["sources"]}
    actual = {}
    all_owners = {}
    for target in ctx.attr.implementations:
        for entry in target[NativeClosure].owners.to_list():
            path, owner = entry.split("|", 1)
            if path in all_owners and all_owners[path] != owner:
                fail("Co-simulation links duplicate implementation owners: " + path)
            all_owners[path] = owner
            if path.startswith(prefix + "src/chrono_vehicle/cosim/"):
                actual[path] = owner
    if actual != expected:
        fail("Co-simulation compile owners differ from the selected original source families")
    out = ctx.actions.declare_file(ctx.label.name + ".sh")
    ctx.actions.write(out, "#!/bin/sh\n# Exact source ownership checked during Bazel analysis.\nexit 0\n", is_executable = True)
    return [DefaultInfo(executable = out)]

cosim_ownership_test = rule(
    implementation = _impl,
    attrs = {
        "implementations": attr.label_list(mandatory = True, aspects = [native_closure_aspect]),
        "with_multicore": attr.bool(default = False),
        "with_sph": attr.bool(default = False),
        "with_dem": attr.bool(default = False),
    },
    test = True,
)
