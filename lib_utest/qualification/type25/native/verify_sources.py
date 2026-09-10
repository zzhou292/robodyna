"""Reuse the existing pinned-source verifier for this native donor inventory."""
from pathlib import Path
import importlib.util

root = Path(__file__).resolve().parent
helper = root.parents[1] / 'nodal_rigid_group' / 'native' / 'verify_sources.py'
spec = importlib.util.spec_from_file_location('tl_pinned_source_verifier', helper)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
module.verify(root)
print('Pinned TYPE25 sources and exact arithmetic fragments verified')
