#!/usr/bin/env python3
"""C++-shape CUDA sources without invoking NVCC or claiming CUDA validity."""
from pathlib import Path
import json
import re
import subprocess
import tempfile


HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
SOURCES = [
    "lib_src/solvers/NodalCinStartup.cpp",
    "lib_src/solvers/NodalAssemblyCinForecast.cpp",
    "lib_src/solvers/NodalCinStorage.cu",
    "lib_src/solvers/cin_advance/ForceGather.cu",
    "lib_src/solvers/cin_advance/ForceTransfers.cu",
    "lib_src/solvers/ExplicitNodalCinStep.cu",
    "lib_src/solvers/FENodalState.cu",
    "lib_utest/qualification/cin_master_gather/CallerTest.cu",
    "lib_utest/qualification/cin_master_gather/OwnerTest.cu",
]


def shaped(source: str) -> str:
    return re.sub(r"<<<.*?>>>", "", source, flags=re.S)


def check(compiler: list[str], source: Path, output: Path) -> None:
    output.write_text(shaped(source.read_text()))
    subprocess.run(
        compiler + ["-I" + str(source.parent), str(output)],
        cwd=ROOT,
        check=True,
    )
    print("PASS CXX/CUDA-shaped syntax", source.relative_to(ROOT), flush=True)


def main() -> None:
    with tempfile.TemporaryDirectory(prefix="cin-master-gather-syntax-") as temporary:
        work = Path(temporary)
        builtins = work / "builtins.h"
        builtins.write_text(
            "#include <cuda_runtime.h>\n"
            "extern dim3 threadIdx,blockIdx,blockDim,gridDim;\n"
            "void __syncthreads();\n"
            "unsigned atomicExch(unsigned*,unsigned);\n"
            "unsigned atomicMin(unsigned*,unsigned);\n"
            "unsigned long long atomicMin(unsigned long long*,unsigned long long);\n"
            "unsigned __ballot_sync(unsigned,int);\n"
            "int __ffs(int);\n"
        )
        solver_headers = ROOT / "lib_src/solvers"
        shadow_lib_src = work / "lib_src"
        shadow_lib_src.mkdir()
        for sibling in (ROOT / "lib_src").iterdir():
            if sibling.name != "solvers":
                (shadow_lib_src / sibling.name).symlink_to(sibling, target_is_directory=sibling.is_dir())
        for header in solver_headers.rglob("*"):
            if header.suffix not in {".h", ".cuh"}:
                continue
            source = header.read_text()
            target = work / "lib_src/solvers" / header.relative_to(solver_headers)
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_text(shaped(source) if "<<<" in source else source)
        compiler = [
            "g++",
            "-std=c++17",
            "-x",
            "c++",
            "-fsyntax-only",
            "-fno-fast-math",
            "-ffp-contract=off",
            "-I" + str(work),
            "-I" + str(work / "lib_src/solvers"),
            "-I" + str(ROOT),
            "-I" + str(ROOT / "lib_src/solvers"),
            "-I/usr/local/cuda/include",
            "-D__global__=",
            "-D__device__=",
            "-D__shared__=",
            "-include",
            str(builtins),
        ]
        for name in SOURCES:
            source = ROOT / name
            check(compiler, source, work / name.replace("/", "__"))
        frozen = work / "Frozen.cu"
        subprocess.run(
            ["python3", "-B", str(HERE / "prepare_reference.py"), str(frozen)],
            cwd=ROOT,
            check=True,
        )
        frozen.write_text(shaped(frozen.read_text()))
        subprocess.run(compiler + ["-I" + str(HERE), str(frozen)], cwd=ROOT, check=True)
        print("PASS CXX/CUDA-shaped syntax generated Frozen.cu", flush=True)
    print(json.dumps({
        "status": "passed",
        "translation_units": len(SOURCES) + 1,
        "cuda_compiler": False,
        "cuda_execution": False,
    }))


if __name__ == "__main__":
    main()
