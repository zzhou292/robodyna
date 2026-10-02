"""Original Python demos depend on actual extensions, not syntax-only placeholders."""

load("@rules_python//python:defs.bzl", "py_binary")

def robodyna_python_demo(name, source, helpers, package, manifest, ros = False):
    py_binary(
        name = name,
        srcs = ["//tools/python_demos:launch.py"],
        main = "//tools/python_demos:launch.py",
        args = [
            "--interpreter", "$(rlocationpath @python_sdk//:bin/python3.10)",
            "--executor", "$(rlocationpath //tools/python_demos:execute.py)",
            "--script", "$(rlocationpath " + source + ")",
            "--package-manifest", "$(rlocationpath " + manifest + ")",
            "--case", native.package_name() + ":" + name,
        ] + (["--ros-node", "$(rlocationpath //src/integrations/ros:node)"] if ros else []),
        data = [source, package, manifest, "@python_sdk//:bin/python3.10", "//tools/python_demos:execute.py"] + helpers + (["//src/integrations/ros:node"] if ros else []),
        deps = ["@rules_python//python/runfiles"],
        tags = ["manual", "original-python-demo", "actual-native-bindings"],
        visibility = ["//visibility:public"],
    )
