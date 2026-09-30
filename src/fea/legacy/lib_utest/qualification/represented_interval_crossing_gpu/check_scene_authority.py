#!/usr/bin/env python3
"""Actual C++17 positive/negative compile gates for native scene authority."""
from pathlib import Path
import argparse
import json
import subprocess

parser = argparse.ArgumentParser()
parser.add_argument("--compiler", required=True)
args = parser.parse_args()
here = Path(__file__).resolve().parent
root = here.parents[2]
command = [args.compiler, "-std=c++17", "-fsyntax-only", "-I", str(root),
           str(here / "SceneAuthority.cpp")]
positive = subprocess.run(command, capture_output=True, text=True, timeout=15)
if positive.returncode:
    raise RuntimeError("Scene authority positive control failed:\n" + positive.stderr)
for macro, name in (("TL_TRY_FORGED_SCENE", "AuthenticatedScene"),
                    ("TL_TRY_FORGED_COHORT", "AuthenticatedNumericCohort")):
    negative = subprocess.run(command + [f"-D{macro}=1"],
                              capture_output=True, text=True, timeout=15)
    if negative.returncode == 0:
        raise RuntimeError(f"Caller forged {name} with a braced private key")
    if name not in negative.stderr or "ConstructionKey" not in negative.stderr or "private" not in negative.stderr.lower():
        raise RuntimeError("Negative control failed for an unrelated reason:\n" + negative.stderr)
print(json.dumps({"status": "passed", "language": "C++17",
                  "positive_control": "compiled", "braced_scene_and_cohort_keys": "rejected_private_constructor",
                  "cuda_execution": False}))
