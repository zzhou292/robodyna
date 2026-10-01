"""Bounded serial generation/parity matrix; launch through run_bounded.py."""

import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess

from tools.bindings.compare_surface import compare
from tools.bindings.run_swig import tool_environment
from tools.bindings.snapshot_inputs import snapshot


PROFILES = {"core": ("Core", ("python", "csharp")),
            "fea": ("Fea", ("python",)),
            "vehicle": ("Vehicle", ("python", "csharp"))}


def generate(inputs, output, sdk, modules):
    output.mkdir()
    receipt = json.loads((sdk / "sdk.json").read_text())
    swig = Path(receipt["executable"])
    if hashlib.sha256(swig.read_bytes()).hexdigest() != receipt["executable_sha256"]:
        raise ValueError("Declared SWIG executable changed")
    source = inputs / "src/compatibility/chrono/src"
    env = tool_environment(dict(os.environ, SWIG_LIB=receipt["SWIG_LIB"]))
    runs = []
    for module in modules:
        suffix, languages = PROFILES[module]
        for language in languages:
            destination = output / module / language
            destination.mkdir(parents=True)
            interface = source / "chrono_swig" / ("chrono_" + language) / f"ChModule{suffix}_{language}.i"
            command = [str(swig), "-c++", "-" + language, "-DCHRONO_FEA",
                       "-I" + str(source), "-I" + str(inputs / "include"),
                       "-I" + str(inputs / "swig_generated"), "-outdir", str(destination),
                       "-o", str(destination / (module + "_wrap.cpp")), str(interface)]
            with (destination / "stdout.log").open("xb") as stdout, (destination / "stderr.log").open("xb") as stderr:
                result = subprocess.run(command, env=env, stdout=stdout, stderr=stderr)
            runs.append({"module": module, "language": language, "command": command, "exit_code": result.returncode})
            if result.returncode:
                (output / "generation.json").write_text(json.dumps({"runs": runs, "passed": False}, indent=2) + "\n")
                raise RuntimeError(f"{module}/{language} generation failed; preserved diagnostics at {destination}")
    (output / "generation.json").write_text(json.dumps({
        "schema": "robodyna.swig_generation_matrix.v1", "passed": True,
        "source_receipt": str(inputs / "parser-inputs.json"), "sdk": receipt,
        "runs": runs, "numpy": False, "fea_enabled": True,
        "scope": "Actual inherited wrapper generation only; no native/managed compilation or runtime",
    }, indent=2) + "\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo", type=Path, required=True)
    parser.add_argument("--sdk", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--baseline-ref", default="26ef28a9d68adf9d170a78ef1e796bff4ef09d30")
    parser.add_argument("--modules", nargs="+", choices=tuple(PROFILES), default=list(PROFILES))
    args = parser.parse_args()
    repo, sdk, output = args.repo.resolve(), args.sdk.resolve(), args.output.absolute()
    output.mkdir()
    for name, reference in (("baseline", args.baseline_ref), ("candidate", None)):
        directory = output / name
        directory.mkdir()
        snapshot(repo, directory / "inputs", reference)
        generate(directory / "inputs", directory / "generated", sdk, args.modules)
    comparisons = {}
    for module in args.modules:
        comparisons[module] = compare(output / "baseline/generated" / module,
                                      output / "candidate/generated" / module, module, PROFILES[module][1],
                                      output / "baseline/inputs", output / "candidate/inputs", True)
    passed = all(row["passed"] for row in comparisons.values())
    (output / "comparison.json").write_text(json.dumps({
        "schema": "robodyna.swig_matrix_comparison.v1", "passed": passed, "modules": comparisons,
        "scope": "Full generated public surfaces and diagnostics; no runtime parity claim",
    }, indent=2, sort_keys=True) + "\n")
    print(json.dumps({"passed": passed, "modules": {name: row["passed"] for name, row in comparisons.items()}}))
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
