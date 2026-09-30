"""Verify shared complete donors, original geometry and this packet adapter."""
from pathlib import Path
import hashlib
import json
import runpy

root = Path(__file__).resolve().parent
qualification = root.parents[1]
manifest = json.loads((root / 'source-manifest.json').read_text())
if manifest['commit'] != 'a62b27e6baa555d222a580d6218867d0be4d70b5':
    raise ValueError('Unexpected OpenRadioss revision')
for record in manifest['shared_files'] + manifest['adapters']:
    data = (root / record['path']).read_bytes()
    if len(data) != record['size'] or hashlib.sha256(data).hexdigest() != record['sha256']:
        raise ValueError('Mapped native packet dependency changed: ' + record['path'])
runpy.run_path(str(qualification / 'qbat_force/native/verify_sources.py'))
runpy.run_path(str(qualification / 'qbat/source_fixture/verify_fixture.py'))
print('Mapped QBAT native geometry/coefficient/scatter receipt and original fixture verified')
