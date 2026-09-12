"""No executable or runtime is loaded by these readelf format controls."""
from pathlib import Path
import runpy

classify = runpy.run_path(str(Path(__file__).resolve().parents[1] / "inspect_imports.py"))["classify"]
cuda13 = """
2: 0000000000000000 0 FUNC GLOBAL DEFAULT UND __cudaGetKernel@libcudart.so.13 (3)
9: 0000000000000000 0 FUNC GLOBAL DEFAULT UND __cudaRegisterFunction@libcudart.so.13 (3)
49: 0000000000000000 0 FUNC GLOBAL DEFAULT UND __cudaLaunchKernel@libcudart.so.13 (3)
"""
assert classify(cuda13)["supported"]
assert classify(cuda13)["launch_imports"] == ["__cudaLaunchKernel"]
assert not classify(cuda13.replace("UND __cudaGetKernel", "23 __cudaGetKernel"))["supported"]
assert classify("0 FUNC GLOBAL DEFAULT UND cudaLaunchKernel\n"
                "0 FUNC GLOBAL DEFAULT UND __cudaRegisterFunction\n")["supported"]
for unsupported in ("cudaGraphLaunch", "cudaLaunchKernelExC", "cudaLaunchKernel_ptsz", "cuLaunchKernel"):
    result = classify(cuda13 + "0 FUNC GLOBAL DEFAULT UND " + unsupported + "@libcuda.so (2)\n")
    assert not result["supported"] and unsupported in result["unsupported_launch_imports"]
assert not classify("")["supported"]
print("PASS: versioned/unversioned imports, missing handle map, graph/driver/extended/PTDS rejection")
