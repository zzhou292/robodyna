"""Execute the retained managed crank/rod demo and observe real native dynamics."""

import json
import math
import os
from pathlib import Path
import re
import sys
import tempfile

from tools.managed.launch import argument_parser, resolve_inputs, run_demo


def main():
    with tempfile.TemporaryDirectory(dir=os.environ.get("TEST_TMPDIR"), prefix="managed-system-") as work:
        output = Path(work) / "run"
        args = argument_parser().parse_args(sys.argv[1:] + ["--output", str(output)])
        inputs = resolve_inputs(args)
        result = run_demo(inputs, output, timeout=30)
        assert result["status"] == "complete" and result["exit_code"] == 0
        assert len(result["native_backend_paths"]) == 1, result
        assert Path(result["native_backend_paths"][0]).resolve() == inputs["native_core"].resolve(), result
        text = (output / "stdout.log").read_text()
        rows = [(float(t), float(x)) for t, x in re.findall(r"Time:\s*(\S+)\s+Body x:\s*(\S+)", text)]
        assert 500 <= len(rows) <= 501, len(rows)
        assert all(math.isfinite(t) and math.isfinite(x) for t, x in rows)
        assert abs(rows[0][0] - 0.01) < 1e-12
        assert 5 - 1e-10 <= rows[-1][0] <= 5.01 + 1e-10
        assert all(abs(rows[i][0] - rows[i - 1][0] - .01) < 1e-10 for i in range(1, len(rows)))
        assert max(x for _, x in rows) - min(x for _, x in rows) > 1.0, rows[-1]
        assert text.rstrip().endswith("Done"), text[-500:]
        # The original program serializes before stepping. This checks that the
        # managed/native archive call really executes, not physical restart.
        archive = output / "ChronoCSharp.json"
        assert archive.stat().st_size > 100
        json.loads(archive.read_text())
        print(json.dumps({"schema": "robodyna.managed_build_system_test.v1", "status": "pass",
                          "sample_count": len(rows), "final_time": rows[-1][0],
                          "body_x_range": max(x for _, x in rows) - min(x for _, x in rows),
                          "native_backend_paths": result["native_backend_paths"],
                          "scope": "Actual original C# demo, one native core, finite moving rigid mechanism; no GUI or GPU claim"}, indent=2))


if __name__ == "__main__":
    main()
