from pathlib import Path
import importlib.util
import runpy

root = Path(__file__).resolve().parent
shared = root.parents[1] / 'nodal_rigid_group/native/verify_sources.py'
spec = importlib.util.spec_from_file_location('native_provenance', shared)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
module.verify(root)
runpy.run_path(str(root.parents[1] / 'type13_recurrence/native/verify_sources.py'), run_name='__main__')
print('TYPE13 resident endpoint loop and shared native recurrence identities verified')
