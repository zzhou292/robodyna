"""Run one declared SWIG action and retain its diagnostics."""

import argparse
import json
import os
from pathlib import Path
import subprocess


def tool_environment(environment):
    """Let a nested Bazel executable select its own declared runfiles bundle."""
    cleaned = dict(environment)
    for name in ("RUNFILES_DIR", "RUNFILES_MANIFEST_FILE", "RUNFILES_MANIFEST_ONLY",
                 "JAVA_RUNFILES", "PYTHON_RUNFILES", "PYTHONPATH", "PYTHONHOME"):
        cleaned.pop(name, None)
    return cleaned


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--swig", required=True)
    parser.add_argument("--language", choices=("python", "csharp"), required=True)
    parser.add_argument("--interface", required=True)
    parser.add_argument("--wrapper", type=Path, required=True)
    parser.add_argument("--directors", type=Path, required=True)
    parser.add_argument("--proxy-dir", type=Path, required=True)
    parser.add_argument("--report", type=Path, required=True)
    parser.add_argument("--include", action="append", default=[])
    args = parser.parse_args()
    args.wrapper.parent.mkdir(parents=True, exist_ok=True)
    args.proxy_dir.mkdir(parents=True, exist_ok=True)
    command = [args.swig, "-c++", "-" + args.language, "-DCHRONO_FEA"]
    command += ["-I" + value for value in args.include]
    command += ["-o", str(args.wrapper), "-oh", str(args.directors),
                "-outdir", str(args.proxy_dir), args.interface]
    result = subprocess.run(command, capture_output=True, text=True, env=tool_environment(os.environ))
    args.report.write_text(json.dumps({"schema": "robodyna.swig_action.v1", "command": command,
                                      "exit_code": result.returncode, "stdout": result.stdout,
                                      "stderr": result.stderr, "numpy": False,
                                      "scope": "Wrapper generation only"}, indent=2) + "\n")
    if result.returncode:
        raise RuntimeError(result.stderr)


if __name__ == "__main__":
    main()
