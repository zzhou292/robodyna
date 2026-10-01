"""Narrow native generic visualization owners; no FE implementation dependency."""

load(":native_component.bzl", "native_component_library")
load(":visual_sources.bzl", "VISUAL_HEADERS", "VISUAL_SOURCES")

def visual_library(name, component, deps):
    """Compile a source-reviewed geometry, material, model or object component."""
    prefix = "//src/compatibility/chrono:"
    native_component_library(
        name = name,
        srcs = [prefix + path for path in VISUAL_SOURCES[component]],
        hdrs = [prefix + path for path in VISUAL_HEADERS[component]],
        deps = deps,
        tags = ["manual", "cpu-only", "visual-boundary"],
    )
