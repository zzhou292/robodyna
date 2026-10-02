"""Admit the actual pinned launcher's plugin-based CLI before launching ranks."""

import json
import os
from pathlib import Path
import subprocess
import sys

from python.runfiles import runfiles
from tools.mpi.environment import mpi_environment


def main():
    resolver = runfiles.Create()
    sdk = Path(resolver.Rlocation(sys.argv[1])).absolute()
    receipt = json.loads(sdk.read_text())
    if receipt["schema"] != "robodyna.mpi_sdk.v1":
        raise RuntimeError("Unexpected MPI SDK receipt")
    root = sdk.parent
    command = [str(root / "prefix/bin/mpirun.openmpi"), "--prefix", str(root / "prefix"), "--help", "launch"]
    result = subprocess.run(command, env=mpi_environment(os.environ, root), capture_output=True, text=True, timeout=15)
    print(result.stdout)
    print(result.stderr)
    if result.returncode != 0 or "--prefix" not in result.stdout or "OpenRTE" not in result.stdout:
        raise RuntimeError("The declared MPI launcher did not load its actual launch-option plugins")
    print("Actual MPI plugin/CLI admission passed; no ranks were launched")


if __name__ == "__main__":
    main()
