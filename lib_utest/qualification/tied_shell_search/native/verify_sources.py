from pathlib import Path
import importlib.util

root = Path(__file__).resolve().parent
helper = root.parents[1] / 'nodal_rigid_group/native/verify_sources.py'
spec = importlib.util.spec_from_file_location('native_provenance', helper)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
module.verify(root)
shared = root.parents[1] / 'native/qeph/verify_sources.py'
spec = importlib.util.spec_from_file_location('shared_provenance', shared)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
module.verify()
print('Tied search donors and shared native constants verified')
