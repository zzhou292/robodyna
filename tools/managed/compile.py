"""Compile declared C# sources with pinned Roslyn, Mono and net472 references."""

import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import tempfile

from tools.bindings.run_swig import tool_environment


def collect_sources(inputs):
    result = []
    names = {}
    for value in inputs:
        path = Path(value)
        files = sorted(path.rglob("*.cs")) if path.is_dir() else [path]
        for source in files:
            if not source.is_file() or source.suffix != ".cs":
                raise ValueError(f"Expected a declared C# source: {source}")
            if source.name in names:
                raise ValueError(f"Duplicate managed source name: {source.name}")
            names[source.name] = source
            result.append(source)
    if not result:
        raise ValueError("No declared managed sources")
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mono", required=True)
    parser.add_argument("--compiler", required=True)
    parser.add_argument("--source", action="append", default=[])
    parser.add_argument("--reference", action="append", default=[])
    parser.add_argument("--define", action="append", default=[])
    parser.add_argument("--globals", type=Path)
    parser.add_argument("--kind", choices=("library", "exe"), required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--receipt", type=Path, required=True)
    args = parser.parse_args()
    if any(not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", value) for value in args.define):
        raise ValueError("Managed availability symbols must be simple identifiers")
    sources = collect_sources(args.source)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    records = [{"path": str(path), "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}
               for path in sources]
    with tempfile.TemporaryDirectory(prefix="robodyna-managed-") as work:
        if args.globals:
            # Preserve the retained helper API and declared template. These paths
            # are relative to the launcher's admitted output/data working tree.
            content = args.globals.read_text().replace("@CHRONO_VERSION@", "10.0.0")
            content = content.replace("@CH_CS_DATA@", "data/").replace("@CH_CS_VEHICLE_DATA@", "data/vehicle/")
            if "@" in content:
                raise ValueError("Unresolved inherited managed configuration placeholder")
            generated = Path(work) / "ChronoGlobals.cs"
            generated.write_text(content)
            sources.append(generated)
        command = [args.mono, "--robodyna-assembly-dir", str(Path(args.compiler).absolute().parent),
                   args.compiler, "-nologo", "-noconfig", "-nostdlib+", "-langversion:7.3",
                   "-target:" + args.kind, "-out:" + str(args.output)]
        command += ["-reference:" + value for value in args.reference]
        command += ["-define:" + value for value in args.define]
        command += [str(path) for path in sources]
        result = subprocess.run(command, capture_output=True, text=True, env=tool_environment(os.environ))
        args.receipt.write_text(json.dumps({
            "schema": "robodyna.managed_compilation.v1", "kind": args.kind,
            "framework": "net472 references", "language_version": "7.3",
            "compiler": args.compiler,
            "compiler_sha256": hashlib.sha256(Path(args.compiler).read_bytes()).hexdigest(),
            "sources": records, "references": args.reference,
            "availability_symbols": args.define,
            "globals_template": str(args.globals) if args.globals else None,
            "exit_code": result.returncode, "stdout": result.stdout, "stderr": result.stderr,
            "scope": "Managed IL compilation; native wrapper/runtime qualification is separate",
        }, indent=2) + "\n")
        if result.returncode:
            raise RuntimeError(result.stdout + result.stderr)


if __name__ == "__main__":
    main()
