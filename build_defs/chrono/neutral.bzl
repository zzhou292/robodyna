"""Real native ownership for unchanged, domain-neutral Chrono implementation."""

load(":native_component.bzl", "native_component_library")
load(":neutral_sources.bzl", "NEUTRAL_HEADERS", "NEUTRAL_SOURCES", "NEUTRAL_HEADER_TARGETS")
load(":source_paths.bzl", "current_source_label")

_SOURCE_ROOT = "//src/compatibility/chrono:"

def neutral_library(name, component, deps):
    """Compile one neutral component without an aggregate header dependency.

    Args:
        name: Real public library target, not an alias to the inherited aggregate.
        component: Key of the reviewed source/header ownership manifest.
        deps: Narrow foundation or variable-block dependencies.
    """
    native_component_library(
        name = name,
        srcs = [current_source_label(path) for path in NEUTRAL_SOURCES[component]],
        hdrs = [_SOURCE_ROOT + path for path in NEUTRAL_HEADERS[component]],
        deps = deps + NEUTRAL_HEADER_TARGETS.get(component, []),
        tags = ["manual", "neutral-mechanics", "cpu-only"],
    )
