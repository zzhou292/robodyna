#!/usr/bin/env python3
"""Compile fixed baseline/current host ABI checks from authenticated reversals."""
from pathlib import Path
import json
import runpy
import subprocess
import tempfile


HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
PROOF = runpy.run_path(str(HERE / "gather_proof.py"))


def compile_check(work: Path, name: str, includes: list[str],
                  layout: int, storage: int) -> None:
    source = work / f"{name}.cpp"
    output = work / f"{name}.exe"
    source.write_text(
        '#include "lib_src/solvers/NodalCinStorage.h"\n'
        f"static_assert(sizeof(tl::fea::nodal_detail::CinLayout) == {layout});\n"
        f"static_assert(sizeof(tl::fea::nodal_detail::CinStorage) == {storage});\n"
        "int main() { return 0; }\n"
    )
    subprocess.run([
        "g++", "-std=c++17", "-fno-fast-math", "-ffp-contract=off",
        *["-I" + include for include in includes],
        "-I/usr/local/cuda/include", str(source), "-o", str(output),
    ], cwd=ROOT, check=True)
    subprocess.run([str(output)], cwd=ROOT, check=True)


def main() -> None:
    with tempfile.TemporaryDirectory(prefix="cin-master-gather-abi-") as temporary:
        work = Path(temporary)
        baseline = work / "baseline/lib_src/solvers"
        baseline.mkdir(parents=True)
        for name in ["NodalCinLayout.h", "NodalCinStorage.h"]:
            path = "lib_src/solvers/" + name
            baseline.joinpath(name).write_text(
                PROOF["restore"](path, (ROOT / path).read_text()))
        compile_check(
            work, "baseline",
            [str(work / "baseline"), str(ROOT), str(ROOT / "lib_src/solvers")],
            400, 672)
        compile_check(
            work, "current",
            [str(ROOT), str(ROOT / "lib_src/solvers")],
            648, 1000)
    print(json.dumps({
        "status": "passed",
        "baseline": {"CinLayout": 400, "CinStorage": 672},
        "current": {"CinLayout": 648, "CinStorage": 1000},
        "cuda_compilation": False,
        "cuda_execution": False,
    }))


if __name__ == "__main__":
    main()
