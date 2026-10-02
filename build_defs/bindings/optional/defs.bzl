"""Thin managed wrappers and shared first-party implementation owners.

Every optional module shares the existing native_core DSO. Lower-level module
DSOs are explicit dynamic dependencies, including the common image utilities.
"""

load("@rules_cc//cc:defs.bzl", "cc_library", "cc_shared_library")
load("//build_defs/bindings:swig.bzl", "swig_module")
load("//build_defs/features:defs.bzl", "BASELINE_ABI_ONLY")
load("//build_defs/bindings/python/profiles:policy.bzl", "IMPLEMENTATION_PROFILE")

_SOURCE = "//src/compatibility/chrono:"
_COPTS = ["-std=c++17", "-O3", "-fPIC", "-fno-fast-math", "-ffp-contract=off", "-Wno-unused-variable"]

def shared_implementation(name, deps, dynamic_deps, user_link_flags = []):
    cc_shared_library(
        name = name,
        deps = deps,
        dynamic_deps = dynamic_deps,
        shared_lib_name = "librobodyna_" + name + ".so",
        user_link_flags = ["-Wl,-soname,librobodyna_" + name + ".so"] + user_link_flags,
        target_compatible_with = IMPLEMENTATION_PROFILE,
        tags = ["manual", "native-bindings"],
    )

def csharp_module(name, stem, implementation, cpp_deps, dynamic_deps, defines = [], local_defines = [], compatibility = BASELINE_ABI_ONLY, native_basename = None):
    swig_module(
        name = name + "_generated",
        module = name,
        language = "csharp",
        interface = _SOURCE + "src/chrono_swig/chrono_csharp/ChModule" + stem + "_csharp.i",
        sources = [_SOURCE + "swig_source_inputs"],
        defines = defines,
        header_deps = cpp_deps,
        target_compatible_with = compatibility,
        tags = ["manual"],
    )
    native.filegroup(name = name + "_proxies", srcs = [":" + name + "_generated"], output_group = "proxy")
    native.filegroup(name = name + "_wrapper_source", srcs = [":" + name + "_generated"], output_group = "wrapper")
    native.filegroup(name = name + "_directors", srcs = [":" + name + "_generated"], output_group = "directors")
    cc_library(
        name = name + "_wrapper",
        srcs = [":" + name + "_wrapper_source"],
        hdrs = [":" + name + "_directors"],
        deps = cpp_deps,
        local_defines = local_defines,
        copts = _COPTS,
        target_compatible_with = compatibility,
        tags = ["manual", "native-bindings"],
    )
    cc_shared_library(
        name = "csharp_" + name,
        deps = [":" + name + "_wrapper"],
        dynamic_deps = [implementation] + dynamic_deps,
        shared_lib_name = native_basename if native_basename else "libchrono_" + name + ".so",
        user_link_flags = ["-Wl,-soname," + (native_basename if native_basename else "libchrono_" + name + ".so")],
        target_compatible_with = compatibility,
        tags = ["manual", "native-bindings"],
    )
