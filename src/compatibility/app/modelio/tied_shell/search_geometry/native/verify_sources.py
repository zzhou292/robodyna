"""Use TL's existing exact donor/fragment verifier for this native-only packet."""
import importlib.util
from pathlib import Path
import sys

root = Path(__file__).resolve().parent
if len(sys.argv) != 2:
    raise SystemExit('Usage: verify_sources.py TL_SOURCE_ROOT')
helper = (Path(sys.argv[1]).resolve() /
          'lib_utest/qualification/nodal_rigid_group/native/verify_sources.py')
spec = importlib.util.spec_from_file_location('native_provenance', helper)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
module.verify(root)
print('Verified six pinned sources and two exact consumed-thickness fragments')
