"""Check the service declaration's actual closure; its System implementation stays mixed."""

load("//build_defs/chrono:dependency_closure.bzl", "NativeClosure", "native_closure_aspect")
load("//build_defs/chrono:neutral_sources.bzl", "NEUTRAL_HEADERS", "NEUTRAL_SOURCES")
load("//build_defs/chrono:source_paths.bzl", "current_source_path")

def _interface_boundary_impl(ctx):
    interface = "//include/robodyna/mechanics:participant_services"
    foundation = "//src/core:foundation"
    allowed_labels = {str(Label(label)): True for label in [
        interface, foundation, "//src/core/configuration:host_headers",
        "//src/compatibility/chrono:neutral_include_root", "@eigen//:eigen",
    ]}
    headers = {"src/compatibility/chrono/" + path: True for path in NEUTRAL_HEADERS["foundation"]}
    headers["include/robodyna/mechanics/RbParticipantServices.h"] = True
    headers["include/robodyna/mechanics/_virtual_includes/participant_services/robodyna/mechanics/RbParticipantServices.h"] = True
    expected = {current_source_path(path): str(Label(foundation)) for path in NEUTRAL_SOURCES["foundation"]}
    closure = ctx.attr.interface[NativeClosure]
    if closure.imports.to_list():
        fail("Service declaration must not import implementation binaries")
    for label in closure.labels.to_list() + closure.link_owners.to_list():
        if label not in allowed_labels:
            fail("Service declaration reaches an undeclared runtime owner: " + label)
    for flag in closure.link_flags.to_list():
        if flag != "-pthread":
            fail("Unreviewed service-declaration linker flag: " + flag)
    for path in closure.headers.to_list():
        if path in headers or path.startswith("src/core/configuration/") or path.startswith("../eigen"):
            continue
        fail("Service declaration leaks a concrete domain/System header: " + path)
    actual = {}
    for entry in closure.owners.to_list():
        path, owner = entry.split("|", 1)
        if path not in expected or owner != expected[path]:
            fail("Service declaration reaches a concrete runtime implementation: " + entry)
        actual[path] = owner
    if actual != expected:
        fail("Service declaration must use exactly the existing foundation compile owners")
    executable = ctx.actions.declare_file(ctx.label.name + ".sh")
    ctx.actions.write(executable, "#!/bin/sh\n# Actual closure verified during Bazel analysis.\nexit 0\n", is_executable = True)
    return [DefaultInfo(executable = executable)]

interface_boundary_test = rule(
    implementation = _interface_boundary_impl,
    attrs = {"interface": attr.label(aspects = [native_closure_aspect], mandatory = True)},
    test = True,
)
