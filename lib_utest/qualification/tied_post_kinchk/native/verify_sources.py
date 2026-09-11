from pathlib import Path
import importlib.util
root=Path(__file__).resolve().parent
helper=root.parents[1]/"nodal_rigid_group/native/verify_sources.py"
spec=importlib.util.spec_from_file_location("kinchk_source_identity",helper)
module=importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
module.verify(root)
print("Complete KINCHK and exact subroutine extent verified")
