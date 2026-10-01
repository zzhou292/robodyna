"""Locate declared runtime tools and construct the existing watchdog invocation."""

import os
from pathlib import Path

from .jsonio import require
from .manifests import GIB


def runtime_file(logical_name, explicit=None):
    if explicit:
        path = Path(explicit).absolute()
    else:
        try:
            from python.runfiles import runfiles
        except ImportError as error:
            raise ValueError("Bazel runfiles unavailable; supply the explicit runtime tool path") from error
        resolver = runfiles.Create()
        located = resolver.Rlocation(logical_name) if resolver else None
        require(located, f"runtime tool is not packaged: {logical_name}")
        path = Path(located).absolute()
    require(path.is_file(), f"runtime tool unavailable: {path}")
    return path


def clean_environment(gpu_index):
    # Explicit case/request data replaces the qualification harness environment.
    environment = {key: value for key, value in os.environ.items()
                   if not key.startswith(("ROBO_", "GTEST_"))
                   and key not in ("LD_PRELOAD", "CUDA_LAUNCH_BLOCKING")}
    environment.update(CUDA_VISIBLE_DEVICES=str(gpu_index), OMP_NUM_THREADS="1",
                       OPENBLAS_NUM_THREADS="1", MKL_NUM_THREADS="1", NUMEXPR_NUM_THREADS="1")
    return environment


def watchdog_environment(environment, guard):
    """Keep the selected unchanged guard's sibling imports under Python safe-path.

    Bazel's interpreter can omit a script's directory from sys.path. Resolve the
    actual selected script so explicit symlink overrides find their own helpers,
    then prepend that directory without dropping worker-module import paths.
    """
    result = dict(environment)
    # A platform watchdog interpreter must locate its own standard library;
    # the product CLI may use a different packaged Python installation.
    for name in ("PYTHONHOME", "PYTHONEXECUTABLE", "__PYVENV_LAUNCHER__"):
        result.pop(name, None)
    directory = str(Path(guard).resolve(strict=True).parent)
    previous = result.get("PYTHONPATH", "")
    result["PYTHONPATH"] = directory + (os.pathsep + previous if previous else "")
    return result


def monitored_command(guard, resources, report, command, interpreter, gpu=False, stop_file=None):
    """One watchdog command shared by simulation and presentation stages."""
    result = [str(interpreter), "-B", str(guard), "--report", str(report),
            "--lock", resources["workstation_lock"], "--cpus", str(resources["cpu_threads"]),
            "--max-rss-gib", str(resources["rss_bytes"] / GIB),
            "--min-available-gib", str(resources["minimum_available_ram_bytes"] / GIB),
            "--timeout", str(resources["timeout_s"])]
    if gpu:
        result += ["--gpu", str(resources["gpu_index"]), "--min-gpu-free-gib",
                   str(resources["minimum_gpu_free_bytes"] / GIB), "--max-gpu-growth-gib",
                   str(resources["maximum_gpu_growth_bytes"] / GIB), "--gpu-process-diagnostics"]
    if stop_file is not None:
        result += ["--cooperative-stop-file", str(stop_file), "--cooperative-stop-grace-seconds",
                   str(resources["stop_grace_s"])]
    return result + ["--", *map(str, command)]


def guard_command(guard, backend, request, request_sha256, resources, output, mode, interpreter):
    command = [backend, "--mode", mode, "--request", request, "--request-sha256", request_sha256]
    return monitored_command(guard, resources, output / "guard.json", command, interpreter, gpu=True,
                             stop_file=output / "stop.requested")
