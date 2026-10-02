"""CPU MPI coupons with declared executables and the existing process guard."""

load("@rules_python//python:defs.bzl", "py_test")

def local_mpi_test(name, program, ranks, timeout_seconds = 120, program_inputs = [], target_compatible_with = []):
    py_test(
        name = name,
        srcs = ["//tools/mpi:TestMain.py"],
        main = "//tools/mpi:TestMain.py",
        args = [
            "--program", "$(rlocationpath " + program + ")",
            "--sdk", "$(rlocationpath @mpi_sdk//:sdk.json)",
            "--guard", "$(rlocationpath @legacy_fea//tools:run_bounded.py)",
            "--ranks", str(ranks), "--timeout-seconds", str(timeout_seconds),
        ] + [value for label in program_inputs for value in ["--program-input", "$(rlocationpath " + label + ")"]],
        data = [program, "@mpi_sdk//:sdk.json", "@mpi_sdk//:runtime", "@legacy_fea//tools:run_bounded.py", "@legacy_fea//tools:bounded_guard_runtime"] + program_inputs,
        deps = ["//tools/mpi:harness"],
        size = "medium",
        timeout = "moderate",
        tags = ["manual", "local", "cpu-only", "mpi-runtime", "exclusive"],
        target_compatible_with = target_compatible_with,
    )
