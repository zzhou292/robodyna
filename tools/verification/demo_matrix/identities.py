"""Pin selected build/guard tools; SDK re-admission stays with their providers."""

import os
from pathlib import Path
import shutil
import subprocess

from tools.dependencies.cuda_math import file_hash


def tool_identities(environment, interpreter):
    requested = {"bazel": environment["bazel"], "watchdog": environment["watchdog"],
                 "watchdog_python": interpreter["path"]}
    guard = Path(environment["watchdog"]).resolve(strict=True)
    for helper in ("bounded_session.py", "bounded_history.py", "bounded_stop.py"):
        requested["watchdog/" + helper] = str(guard.parent / helper)
    for key in ("CC", "CXX", "AR", "LD"):
        value = os.environ.get(key) or shutil.which({"CC": "gcc", "CXX": "g++", "AR": "ar", "LD": "ld"}[key])
        if not value:
            raise ValueError("Missing selected host tool " + key)
        if not Path(value).is_absolute():
            value = shutil.which(value) or value
        requested[key] = value
    cuda = environment.get("action_environment", {}).get("CUDACXX")
    if cuda:
        requested["nvcc"] = cuda
        directory = Path(cuda).resolve().parent
        for helper in (directory / "ptxas", directory / "fatbinary", directory / "nvlink", directory.parent / "nvvm/bin/cicc"):
            if helper.is_file():
                requested["cuda/" + helper.name] = str(helper)
    # Record the real compiler backend selected by GCC; do not mistake its small
    # driver executable for the complete selected compiler program.
    for key, program in (("CC", "cc1"), ("CXX", "cc1plus")):
        probe = subprocess.run([requested[key], "-print-prog-name=" + program], capture_output=True, text=True, timeout=10, check=True)
        value = probe.stdout.strip()
        if not Path(value).is_absolute():
            value = shutil.which(value) or value
        if Path(value).is_file():
            requested[program] = value
    return {"tools": {name: {"path": str(Path(path).resolve(strict=True)), "sha256": file_hash(Path(path).resolve(strict=True))}
                      for name, path in sorted(requested.items())},
            "scope": "Selected guard/Bazel/Python/CUDA executables and ambient host compiler tools; SDK contents are re-admitted by every resumed Bazel build, not a claim of a hermetic system runtime"}
