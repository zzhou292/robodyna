"""Verify one compile owner for the retained distributed implementation."""

load("//build_defs/chrono:dependency_closure.bzl", "NativeClosure", "native_closure_aspect")
load(":sources.bzl", "DISTRIBUTED_SOURCES")

def _impl(ctx):
    expected = {"src/compatibility/chrono/" + path: str(Label("//src/distributed:synchronization")) for path in DISTRIBUTED_SOURCES}
    all_owners = {}
    actual = {}
    for entry in ctx.attr.implementation[NativeClosure].owners.to_list():
        path, owner = entry.split("|", 1)
        if path in all_owners and all_owners[path] != owner:
            fail("Distributed module acquired duplicate physics/source owners: " + path)
        all_owners[path] = owner
        if path.startswith("src/compatibility/chrono/src/chrono_synchrono/"):
            actual[path] = owner
    if actual != expected:
        fail("Distributed source closure differs from its 34 original translation units")
    for imported in ctx.attr.implementation[NativeClosure].imports.to_list():
        if "ros_sdk" in imported:
            fail("Distributed module must not import the incompatible ROS DDS SDK")
    executable = ctx.actions.declare_file(ctx.label.name + ".sh")
    ctx.actions.write(executable, "#!/bin/sh\n# Actual compile ownership checked during analysis.\nexit 0\n", is_executable = True)
    return [DefaultInfo(executable = executable)]

distributed_ownership_test = rule(
    implementation = _impl,
    attrs = {"implementation": attr.label(mandatory = True, aspects = [native_closure_aspect])},
    test = True,
)
