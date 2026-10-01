"""Launch the isolated binding probe with the declared matching CPython SDK."""

import subprocess
import sys
from python.runfiles import runfiles


if __name__ == "__main__":
    resolver = runfiles.Create()
    paths = [resolver.Rlocation(path) for path in sys.argv[1:]]
    if len(paths) != 5 or not all(paths):
        raise RuntimeError("Declared Python binding test inputs are missing")
    interpreter, script, proxy, extension, backend = paths
    result = subprocess.run([interpreter, "-I", "-X", "faulthandler", script, "--proxy", proxy,
                             "--extension", extension, "--backend", backend], timeout=120)
    raise SystemExit(result.returncode)
