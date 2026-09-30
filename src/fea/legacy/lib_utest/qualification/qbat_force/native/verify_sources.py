"""Read-only verification of complete donors and exact selected fragments."""
from pathlib import Path
import importlib.util
import runpy

root = Path(__file__).resolve().parent
qualification = root.parents[1]
shared = qualification / 'nodal_rigid_group/native/verify_sources.py'
spec = importlib.util.spec_from_file_location('native_provenance', shared)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
module.verify(root)
module.verify(qualification / 'native/law44_one_point')
runpy.run_path(str(qualification / 'qbat/native/verify_sources.py'))
runpy.run_path(str(qualification / 'native/qeph/verify_sources.py'))['verify']()
print('QBAT force donors/fragments and shared current geometry verified')
