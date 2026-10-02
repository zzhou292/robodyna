"""Legacy import spelling for declared Robodyna native binding artifacts.

Generated SWIG proxies still refer to pychrono.core. Keep those exact module
identities instead of loading another copy under the product-facing name.
"""

import ctypes as _ctypes
import importlib as _importlib
import json as _json
import sys as _sys
from pathlib import Path as _Path

# Do not resolve __file__: this initializer is intentionally a build symlink.
_package_root = _Path(__file__).parent.parent
_manifest = _json.loads((_package_root / "package.json").read_text())
if _manifest.get("schema") != "robodyna.python_binding_package.v1":
    raise ImportError("Unsupported Robodyna binding package manifest")
if _sys.implementation.name != "cpython" or _sys.version_info[:2] != (3, 10):
    raise ImportError("This native Robodyna binding package requires CPython 3.10")

# A single interpreter must choose one generated binding package. Loading a
# second package after changing sys.path could mix incompatible wrapper layouts
# against an already-loaded SONAME. Keep the admission marker for process life,
# including after an import failure, because native DSOs may already be loaded.
_active_root = str(_package_root.resolve())
_previous_root = getattr(_sys, "_robodyna_binding_package_root", None)
if _previous_root is not None and _previous_root != _active_root:
    raise ImportError("Robodyna cannot mix binding packages in one interpreter; start a separate process")
_sys._robodyna_binding_package_root = _active_root

for _relative in _manifest.get("runtime_python_roots", []):
    _runtime_root = (_package_root / _relative).resolve(strict=True)
    if not _runtime_root.is_dir():
        raise ImportError("Declared Python runtime root is not a directory")
    _sys.path.insert(0, str(_runtime_root))

# Some upstream SDK DSOs have nontransitive RUNPATHs. Their provider publishes
# the authenticated, required dependency order; load those exact original files
# before native implementation owners. No ambient loader search path is changed.
_sdk_native_handles = []
_sdk_native_paths = {}
for _relative in _manifest.get("runtime_python_roots", []):
    _marker = _package_root / _relative / ".robodyna-runtime.json"
    if not _marker.is_file():
        continue
    _runtime = _json.loads(_marker.read_text())
    if _runtime.get("schema") != "robodyna.python_runtime_root.v1":
        raise ImportError("Unsupported declared SDK runtime marker")
    for _library in _runtime.get("native_libraries", []):
        _path = _Path(_library["path"]).resolve(strict=True)
        _soname = _library["soname"]
        if _soname in _sdk_native_paths:
            if _sdk_native_paths[_soname] != _path:
                raise ImportError("Conflicting native SDK owners for " + _soname)
            continue
        _sdk_native_handles.append(_ctypes.CDLL(str(_path), mode=_ctypes.RTLD_GLOBAL))
        _sdk_native_paths[_soname] = _path

# Keeping these handles alive also preserves the shared class-factory owner.
# Extension DT_NEEDED uses these same SONAMEs; it does not add a static core.
_native_handles = [
    _ctypes.CDLL(str(_package_root / item["path"]), mode=_ctypes.RTLD_GLOBAL)
    for item in _manifest["backends"]
]
_module_objects = {}
for _item in _manifest["modules"]:
    _name = _item["name"]
    _module = _importlib.import_module(__name__ + "." + _name)
    _module_objects[_name] = _module
    globals()[_name] = _module

# Preserve the original root-level core convenience API without swallowing
# missing optional extensions as the historical local-install template did.
for _name in dir(core):
    if not _name.startswith("_"):
        globals()[_name] = getattr(core, _name)

__all__ = sorted(name for name in globals() if not name.startswith("_"))
