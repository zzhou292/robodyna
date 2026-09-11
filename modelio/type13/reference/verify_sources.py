"""Use the existing TL pinned-source verifier for this app conversion inventory."""
from pathlib import Path
import importlib.util
import sys

helper = Path(sys.argv[1]) / 'lib_utest/qualification/nodal_rigid_group/native/verify_sources.py'
spec = importlib.util.spec_from_file_location('type13_converter_source_verifier', helper)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
module.verify(Path(__file__).resolve().parent)
print('Pinned TYPE13 source converter/default donors verified')
