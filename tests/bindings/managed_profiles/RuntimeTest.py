"""Execute an actual managed/native host interop test with no GPU constructors."""
import json
import os
from pathlib import Path
import sys
import tempfile
from tools.managed.launch import argument_parser, resolve_inputs, run_demo

parser = argument_parser()
parser.add_argument("--expect-marker", required=True)
with tempfile.TemporaryDirectory(dir=os.environ.get("TEST_TMPDIR"), prefix="managed-profile-") as directory:
    output = Path(directory) / "run"
    args = parser.parse_args(sys.argv[1:] + ["--output", str(output)])
    os.environ["CUDA_VISIBLE_DEVICES"] = ""
    result = run_demo(resolve_inputs(args), output, 30)
    text = (output / "stdout.log").read_text()
    if args.expect_marker not in text:
        raise RuntimeError("Managed profile check did not complete:\n" + text)
    print(text)
    print(json.dumps({"schema": "robodyna.managed_profile_interop.v1", "passed": True,
                      "marker": args.expect_marker,
                      "scope": "Actual headless managed/native ownership and values; no original GUI loop, camera construction, rendering, ROS transport or GPU execution"}))
