"""Run the original finite Python functions demo through the actual native package."""

import json
import math
import os
from pathlib import Path
import subprocess
import sys
from python.runfiles import runfiles


def main():
    resolver = runfiles.Create()
    interpreter, executor, source, manifest = [resolver.Rlocation(value) for value in sys.argv[1:5]]
    if not all((interpreter, executor, source, manifest)):
        raise RuntimeError("Declared original-demo inputs are missing")
    root = Path(os.environ["TEST_TMPDIR"])
    data = root / "empty-data"
    data.mkdir()
    output = root / "functions-run"
    subprocess.run([interpreter, "-I", "-B", executor, "--script", source,
                    "--package-manifest", manifest, "--case", "core/functions",
                    "--data-root", str(data), "--output-dir", str(output)], check=True, timeout=60)
    result = json.loads((output / "result.json").read_text())
    if result["exit_code"] != 0 or not result["script_started"]:
        raise RuntimeError("The original demo did not complete")
    directory = output / "DEMO_OUTPUT/Functions"
    expected = {"f_sine.out": 101, "f_test.out": 101, "f_seq.out": 101, "f_rep.out": 1001}
    rows = {}
    for name, count in expected.items():
        rows[name] = [[float(value) for value in line.split()] for line in (directory / name).read_text().splitlines()]
        if len(rows[name]) != count or any(len(row) != 4 or not all(map(math.isfinite, row)) for row in rows[name]):
            raise RuntimeError("Original function output is incomplete/nonfinite: " + name)
    if abs(rows["f_sine.out"][25][1] + 2) > 1e-6:
        raise RuntimeError("Native sine evaluation differs at x=0.5")
    if rows["f_test.out"][0][1] != 1 or rows["f_test.out"][50][1] != -1:
        raise RuntimeError("Original Python director callback values differ")
    print(json.dumps({"schema": "robodyna.original_python_functions.v1", "passed": True,
                      "source": source, "data_rows": expected, "scope": "One finite original demo; no graphics/GPU/vehicle runtime claim"}))


if __name__ == "__main__":
    main()
