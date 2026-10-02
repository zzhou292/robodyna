"""Named package compositions over configured native module owners."""

load("//bindings/python:package.bzl", "python_binding_package")
load("//build_defs/bindings/python/profiles:policy.bzl", "IMPLEMENTATION_PROFILE", "PROFILE_IDENTITY", "ROS_IDENTITY")

def profile_package(name, modules, backends, compatibility):
    python_binding_package(
        name = name,
        numpy = True,
        python_runtime_anchor = "@numpy_sdk//:site-packages/numpy/__init__.py",
        runtime_data = ["@numpy_sdk//:runtime_files"],
        profile = name,
        abi_capabilities = PROFILE_IDENTITY + ROS_IDENTITY,
        proxies = {module: "//build_defs/bindings/python/profiles:" + module + "_proxy" for module in modules},
        extensions = {module: "//build_defs/bindings/python/profiles:python_" + module for module in modules},
        backends = ["//build_defs/bindings:native_core"] + backends,
        target_compatible_with = IMPLEMENTATION_PROFILE + compatibility,
        tags = ["manual", "native-python-package", "explicit-binding-profile"],
    )
    native.filegroup(name = name + "_manifest", srcs = [":" + name], output_group = "manifest")
