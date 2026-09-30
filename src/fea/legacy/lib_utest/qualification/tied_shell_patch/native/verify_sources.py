from pathlib import Path
import importlib.util

root = Path(__file__).resolve().parent
shared = root.parents[1] / 'nodal_rigid_group/native/verify_sources.py'
spec = importlib.util.spec_from_file_location('native_provenance', shared)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
module.verify(root)
print('TYPE2 kinematic force and motion original sources and exact fragments verified')
