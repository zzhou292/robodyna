"""Verify shared complete donors and exact new caller fragments without copies."""
from pathlib import Path
import runpy
import importlib.util

root = Path(__file__).resolve().parent
shared = root.parents[1] / 'tied_shell_search/native/verify_sources.py'
runpy.run_path(str(shared), run_name='__main__')
helper = root.parents[1] / 'nodal_rigid_group/native/verify_sources.py'
spec = importlib.util.spec_from_file_location('bucket_provenance', helper)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
module.verify(root)
print('Native bucket shared donors and exact domain/coordinate fragments verified')
