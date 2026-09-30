"""Read ELF imports without executing the target or loading CUDA.

This checks the named ELF only. Repeat for application DSOs that own kernels;
it cannot discover dynamically loaded plugins or unobserved runtime calls.
"""
import json
from pathlib import Path
import re
import subprocess
import sys

def classify(symbols):
    # GNU readelf may append a version index after the symbol, e.g. "(3)".
    imports = {match.group(1).split("@")[0]
               for match in re.finditer(r"\bUND\s+(\S+)", symbols)}
    launches = sorted(name for name in imports if re.search(r"(?:cuda|cu).*(?:Launch|launch)", name))
    supported = {"cudaLaunchKernel", "__cudaLaunchKernel"}
    unsupported = [name for name in launches if name not in supported]
    mapping = "__cudaRegisterFunction" in imports and (
        "__cudaLaunchKernel" not in imports or "__cudaGetKernel" in imports)
    return {"schema": "robo_dyna.cuda_kernel_imports.v1", "scope": "direct ELF imports only",
            "launch_imports": launches, "unsupported_launch_imports": unsupported,
            "registered_mapping_imports": mapping,
            "supported": bool(launches) and not unsupported and mapping}


if __name__ == "__main__":
    binary = Path(sys.argv[1]).resolve(strict=True)
    symbols = subprocess.check_output(["readelf", "--dyn-syms", "-W", str(binary)], text=True)
    result = dict(classify(symbols), binary=str(binary))
    print(json.dumps(result, indent=2))
    sys.exit(0 if result["supported"] else 1)
