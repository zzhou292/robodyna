"""Run real cross-module C# values/lifetimes against the selected shared owners."""
import os
from pathlib import Path
import sys
import tempfile
from tools.managed.launch import argument_parser, resolve_inputs, run_demo

with tempfile.TemporaryDirectory(dir=os.environ.get("TEST_TMPDIR"), prefix="managed-modules-") as directory:
    output = Path(directory) / "run"
    args = argument_parser().parse_args(sys.argv[1:] + ["--output", str(output)])
    result = run_demo(resolve_inputs(args), output, 30)
    text = (output / "stdout.log").read_text()
    if result["exit_code"] or "ROBODYNA MANAGED MODULES PASS" not in text:
        raise RuntimeError(text)
    print(text)
