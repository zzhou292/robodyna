"""Check unchanged adapter compilation and reuse of the existing solvers."""

load("//build_defs/chrono:dependency_closure.bzl", "NativeClosure", "native_closure_aspect")
load(":sources.bzl", "PRECICE_GROUPS")

def _impl(ctx):
    prefix = "src/compatibility/chrono/"
    expected = {prefix + p: str(Label("//src/coupling/precice:" + name)) for name, group in PRECICE_GROUPS.items() for p in group["sources"]}
    found = {}
    owners = {}
    for target in ctx.attr.implementations:
        for entry in target[NativeClosure].owners.to_list():
            path, owner = entry.split("|", 1)
            if path in owners and owners[path] != owner:
                fail("Duplicate preCICE/solver compile owner: " + path)
            owners[path] = owner
            if path.startswith(prefix + "src/chrono_precice/"):
                found[path] = owner
    if found != expected:
        fail("Retained preCICE source closure differs")
    out = ctx.actions.declare_file(ctx.label.name + ".sh")
    ctx.actions.write(out, "#!/bin/sh\n# Actual source-owner graph checked during analysis.\nexit 0\n", is_executable = True)
    return [DefaultInfo(executable = out)]

precice_ownership_test = rule(
    implementation = _impl,
    attrs = {"implementations": attr.label_list(mandatory = True, aspects = [native_closure_aspect])},
    test = True,
)
