"""Compile and execute a local-function program from copied declared SDK files."""

import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile

from python.runfiles import runfiles
from tools.dependencies.cuda_math import file_hash
from tools.managed.environment import mono_environment


def copy_inputs(root, destination, files):
    for row in files:
        relative = row.get("output", row["path"])
        source = root / relative
        if file_hash(source) != row["sha256"]:
            raise RuntimeError("Declared SDK file differs from its pin: " + relative)
        target = destination / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, target)


def main():
    resolver = runfiles.Create()
    if len(sys.argv) != 4 or resolver is None:
        raise RuntimeError("Expected fixture and both declared SDK receipts")
    paths = [Path(resolver.Rlocation(value)).absolute() for value in sys.argv[1:]]
    source, mono_receipt, compiler_receipt = paths
    mono = json.loads(mono_receipt.read_text())
    compiler = json.loads(compiler_receipt.read_text())
    parent = os.environ.get("TEST_UNDECLARED_OUTPUTS_DIR") or os.environ.get("TEST_TMPDIR")
    work = Path(tempfile.mkdtemp(dir=parent, prefix="compiler-admission-"))
    mono_root = work / "mono"
    compiler_root = work / "compiler"
    # Physical copies ensure the compiler cannot accidentally discover an
    # undeclared assembly beside a symlink's original SDK installation.
    copy_inputs(mono_receipt.parent, mono_root, mono["files"])
    copy_inputs(compiler_receipt.parent, compiler_root, compiler["files"])
    copy_inputs(compiler_receipt.parent, compiler_root, compiler["runtime_overlay"]["files"])
    fixture = work / "LocalFunctions.cs"
    shutil.copyfile(source, fixture)
    executable = work / "Robodyna.CompilerAdmission.exe"
    tool = mono_root / "usr/bin/mono-sgen"
    compiler_path = compiler_root / compiler["compiler"]
    environment = mono_environment(os.environ, mono_root, [compiler_path.parent])
    commands = [
        [str(tool), "--config", str(mono_root / "etc/mono/config"), str(compiler_path),
         "-nologo", "-noconfig", "-nostdlib+", "-langversion:7.3",
         "-reference:" + str(mono_root / "usr/lib/mono/4.7.2-api/mscorlib.dll"),
         "-doc:" + str(work / "LocalFunctions.xml"), "-out:" + str(executable), str(fixture)],
        [str(tool), "--config", str(mono_root / "etc/mono/config"), str(executable)],
    ]
    receipts = []
    try:
        for phase, command in zip(("compile", "execute"), commands):
            result = subprocess.run(command, cwd=work, env=environment, capture_output=True, text=True, timeout=30)
            receipts.append({"phase": phase, "command": command, "exit_code": result.returncode,
                             "stdout": result.stdout, "stderr": result.stderr})
            if result.returncode:
                raise RuntimeError(result.stdout + result.stderr)
        if "ROBODYNA COMPILER ADMISSION PASS" not in receipts[-1]["stdout"]:
            raise RuntimeError("Compiled program did not publish its expected result")
        if not (work / "LocalFunctions.xml").is_file():
            raise RuntimeError("Documentation code path was not exercised")
        print("PASS: copied declared compiler/runtime closure, C#7.3 local functions and actual execution")
    finally:
        (work / "receipt.json").write_text(json.dumps({"scope": "Compiler/runtime admission; no simulation claim",
                                                       "phases": receipts}, indent=2) + "\n")
        print("Compiler admission evidence:", work)


if __name__ == "__main__":
    main()
