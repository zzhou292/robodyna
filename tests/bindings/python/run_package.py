"""Run the actual binding package check using the admitted CPython interpreter."""

import subprocess
import sys
from python.runfiles import runfiles


def main():
    resolver = runfiles.Create()
    interpreter, script, manifest = [resolver.Rlocation(value) for value in sys.argv[1:4]]
    if not all((interpreter, script, manifest)):
        raise RuntimeError("Declared Python package test inputs are missing")
    result = subprocess.run([interpreter, "-I", "-B", "-X", "faulthandler", script,
                             "--package-manifest", manifest] + sys.argv[4:], timeout=60)
    raise SystemExit(result.returncode)


if __name__ == "__main__":
    main()
