"""Verify complete native donors, exact fragments, and the original triangle authority."""
from pathlib import Path
import hashlib
import importlib.util
import json
import runpy

root = Path(__file__).resolve().parent
qualification = root.parents[1]
spec = importlib.util.spec_from_file_location(
    'one_point_provenance', qualification / 'nodal_rigid_group/native/verify_sources.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
module.verify(root)
module.verify(qualification / 'native/law44_one_point')
fixture = qualification / 'qbat/source_fixture'
assert hashlib.sha256((fixture / 'YarisQbatSourceFixture.h').read_bytes()).hexdigest() == \
    '87c3902373d3a5e9e27c09522ad9adc5f4c6e6822eb2f64aa613d82bc1638c98'
assert hashlib.sha256((fixture / 'source-manifest.json').read_bytes()).hexdigest() == \
    '1c6362e1900eef24cb854189fd2448ae2d6efb073fdd9414b5554c0e4dcda1e2'
manifest = json.loads((fixture / 'source-manifest.json').read_text())
assert manifest['part_id'] == 2000524 and manifest['thickness_m'] == .0005
assert manifest['excluded_from_quad_fixture'] == [dict(element_id=2357656, canonical_index=228325,
    source_line=274492, raw_record=[2357656,2000524,2300357,2300138,2300139,2300139])]
runpy.run_path(str(qualification / 'native/t3/verify_sources.py'))['verify']()
runpy.run_path(str(qualification / 'qbat_force/native/verify_sources.py'))
print('One-point T3 donors/fragments, shared material, and original source fixture verified')
