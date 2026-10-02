"""Actual optional CPython wrappers over the existing shared native owners."""

load("@rules_cc//cc:defs.bzl", "cc_library", "cc_shared_library")
load("//build_defs/bindings:swig.bzl", "swig_module")
load("//build_defs/features:defs.bzl", "BASELINE_ABI_ONLY")

_SOURCE = "//src/compatibility/chrono:"
_COPTS = ["-std=c++17", "-O3", "-fPIC", "-fno-fast-math", "-ffp-contract=off", "-Wno-unused-variable"]

def python_module(name, stem, implementation, cpp_deps, dynamic_deps, defines = [], numpy = False, compatibility = BASELINE_ABI_ONLY, compile_defines = [], copts = []):
    """Generate and compile a retained interface without statically owning its backend."""
    array_deps = ["@numpy_sdk//:headers"] if numpy else []
    array_defines = ["CHRONO_PYTHON_NUMPY"] if numpy else []
    swig_module(
        name = name + "_generated",
        module = name,
        language = "python",
        interface = _SOURCE + "src/chrono_swig/chrono_python/ChModule" + stem + "_python.i",
        sources = [_SOURCE + "swig_source_inputs"],
        defines = defines + array_defines,
        header_deps = cpp_deps + array_deps,
        target_compatible_with = compatibility,
        tags = ["manual"],
    )
    native.filegroup(name = name + "_proxy", srcs = [":" + name + "_generated"], output_group = "proxy")
    native.filegroup(name = name + "_wrapper_source", srcs = [":" + name + "_generated"], output_group = "wrapper")
    native.filegroup(name = name + "_directors", srcs = [":" + name + "_generated"], output_group = "directors")
    cc_library(
        name = name + "_wrapper",
        srcs = [":" + name + "_wrapper_source"],
        hdrs = [":" + name + "_directors"],
        deps = cpp_deps + ["@python_sdk//:headers"] + array_deps,
        local_defines = array_defines + compile_defines,
        copts = _COPTS + copts,
        target_compatible_with = compatibility,
        tags = ["manual", "native-python-bindings"],
    )
    cc_shared_library(
        name = "python_" + name,
        deps = [":" + name + "_wrapper"],
        dynamic_deps = [implementation] + dynamic_deps,
        shared_lib_name = "_" + name + ".so",
        user_link_flags = ["-Wl,-soname,_" + name + ".so"],
        target_compatible_with = compatibility,
        tags = ["manual", "native-python-bindings"],
    )
