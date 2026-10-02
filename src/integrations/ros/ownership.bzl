"""Check actual ROS compile/link separation using the shared closure aspect."""

load("//build_defs/chrono:dependency_closure.bzl", "NativeClosure", "native_closure_aspect")
load(":sources.bzl", "ROS_GROUPS")

def _ownership_impl(ctx):
    prefix = "src/compatibility/chrono/"
    expected = {}
    groups = {
        "protocol": "protocol", "simulation": "simulation", "handlers": "simulation",
        "vehicle": "vehicle_handlers", "robot": "robot_handlers", "node": "node_backend",
    }
    if ctx.attr.sensor:
        groups["sensor"] = "sensor_handlers"
    if ctx.attr.rendered_sensor:
        groups["rendered_sensor"] = "sensor_handlers"
    for group, owner in groups.items():
        for path in ROS_GROUPS[group]["sources"]:
            expected[prefix + path] = str(Label("//src/integrations/ros:" + owner))
    found = {}
    for target in ctx.attr.owners:
        closure = target[NativeClosure]
        for entry in closure.owners.to_list():
            path, owner = entry.split("|", 1)
            if path.startswith(prefix + "src/chrono_ros/"):
                if expected.get(path) != owner:
                    fail("Wrong ROS source owner: " + entry)
                if path in found and found[path] != owner:
                    fail("ROS source compiled twice: " + path)
                found[path] = owner
    if found != expected:
        fail("ROS ownership graph omits an admitted implementation unit")
    for entry in ctx.attr.node[NativeClosure].owners.to_list():
        path = entry.split("|", 1)[0]
        if path.startswith(prefix) and not path.startswith(prefix + "src/chrono_ros/"):
            fail("Middleware node acquired a mechanics implementation: " + entry)
    for label in ctx.attr.simulation[NativeClosure].imports.to_list():
        if "ros_sdk" in label:
            fail("Production simulation library imported ROS middleware: " + label)
    executable = ctx.actions.declare_file(ctx.label.name + ".sh")
    ctx.actions.write(executable, "#!/bin/sh\n# Actual compile/link ownership checked during analysis.\nexit 0\n", is_executable = True)
    return [DefaultInfo(executable = executable)]

ros_ownership_test = rule(
    implementation = _ownership_impl,
    attrs = {
        "sensor": attr.bool(default = False),
        "rendered_sensor": attr.bool(default = False),
        "owners": attr.label_list(mandatory = True, aspects = [native_closure_aspect]),
        "node": attr.label(mandatory = True, aspects = [native_closure_aspect]),
        "simulation": attr.label(mandatory = True, aspects = [native_closure_aspect]),
    },
    test = True,
)
