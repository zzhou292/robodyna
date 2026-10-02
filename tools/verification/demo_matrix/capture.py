"""Capture expanded and public Bazel query snapshots inside the outer guard."""

import argparse
import json
from pathlib import Path
import subprocess

from tools.verification.chrono_inventory import digest
from tools.verification.demo_matrix.query import PUBLIC_EXPRESSION, QUERY_EXPRESSION
from tools.verification.demo_matrix.roster import build_metadata


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--request", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    request = json.loads(args.request.read_text())
    args.output.mkdir(parents=True, exist_ok=False)
    receipt = {"schema": "robodyna.demo_query_capture.v1", "status": "failed", "queries": [],
               "build_metadata": build_metadata(request["repository"])}
    try:
        for name, expression, format_name in [("expanded", QUERY_EXPRESSION, "xml"),
                                              ("public", PUBLIC_EXPRESSION, "label")]:
            command = request["bazel_prefix"] + ["query", expression, "--output=" + format_name,
                                                "--noimplicit_deps"] + request["repository_options"]
            output = args.output / (name + (".xml" if format_name == "xml" else ".txt"))
            errors = args.output / (name + ".stderr.log")
            with output.open("x") as stdout, errors.open("x") as stderr:
                completed = subprocess.run(command, cwd=request["repository"], stdout=stdout, stderr=stderr, check=False)
            receipt["queries"].append({"command": command, "exit_code": completed.returncode,
                                       "output": output.name, "sha256": digest(output.read_bytes()),
                                       "bytes": output.stat().st_size, "stderr": errors.name})
            print(name, "exit", completed.returncode, "bytes", output.stat().st_size, flush=True)
            if completed.returncode:
                raise RuntimeError("Bazel query failed; original diagnostics preserved in " + str(errors))
        if receipt["build_metadata"] != build_metadata(request["repository"]):
            raise RuntimeError("Build declarations changed during query capture")
        receipt["status"] = "passed"
    finally:
        (args.output / "receipt.json").write_text(json.dumps(receipt, indent=2) + "\n")


if __name__ == "__main__":
    main()
