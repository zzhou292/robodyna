"""Run the actual coupled binding test with explicitly declared CPython inputs."""

import subprocess
import sys
from python.runfiles import runfiles


if __name__ == "__main__":
    resolver = runfiles.Create()
    paths = [resolver.Rlocation(path) for path in sys.argv[1:]]
    if len(paths) != 8 or not all(paths):
        raise RuntimeError("Declared Python FEA binding inputs are missing")
    interpreter, script, core_proxy, core_extension, fea_proxy, fea_extension, backend, checks = paths
    result = subprocess.run([
        interpreter, "-I", "-X", "faulthandler", script,
        "--core-proxy", core_proxy, "--core-extension", core_extension,
        "--fea-proxy", fea_proxy, "--fea-extension", fea_extension,
        "--backend", backend, "--core-check", checks,
    ], timeout=120)
    raise SystemExit(result.returncode)
