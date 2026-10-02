"""Child environments for the declared Mono SDK, without installed-SDK fallback."""

import os
from pathlib import Path

from tools.bindings.run_swig import tool_environment


def mono_environment(environment, sdk_root, assembly_directories=(), native_directories=()):
    root = Path(sdk_root).absolute()
    cleaned = tool_environment(environment)
    for name in tuple(cleaned):
        if name.startswith("MONO_") or name in ("LD_LIBRARY_PATH", "LD_PRELOAD"):
            cleaned.pop(name)
    cleaned.update({
        "MONO_PATH": os.pathsep.join([str(root / "usr/lib/mono/4.5")] +
                                    [str(Path(path).absolute()) for path in assembly_directories]),
        "MONO_CFG_DIR": str(root / "etc"),
        "MONO_GAC_PREFIX": str(root / "usr"),
        "LD_LIBRARY_PATH": os.pathsep.join([str(root / "usr/lib")] +
                                          [str(Path(path).absolute()) for path in native_directories]),
        "LC_ALL": "C",
    })
    return cleaned
