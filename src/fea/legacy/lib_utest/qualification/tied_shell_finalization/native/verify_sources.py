"""Verify complete finalizer and shared native registration sources."""
from pathlib import Path
import importlib.util
import runpy
root = Path(__file__).resolve().parent
runpy.run_path(str(root.parents[1]/'tied_shell_classification/native/verify_sources.py'), run_name='__main__')
helper = root.parents[1]/'nodal_rigid_group/native/verify_sources.py'
spec = importlib.util.spec_from_file_location('finalizer_provenance', helper)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
module.verify(root)
print('Complete I2TID3 and selected source initialization donors verified')
