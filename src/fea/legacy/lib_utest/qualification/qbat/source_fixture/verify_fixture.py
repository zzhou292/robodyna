"""Authenticate the root-generated original 4250-quad geometry fixture."""
from pathlib import Path
import hashlib
import json

root = Path(__file__).resolve().parent
raw = (root / 'source-manifest.json').read_bytes()
assert hashlib.sha256(raw).hexdigest() == '1c6362e1900eef24cb854189fd2448ae2d6efb073fdd9414b5554c0e4dcda1e2'
manifest = json.loads(raw)
data = (root / manifest['file']).read_bytes()
assert len(data) == manifest['bytes'] == 659264
assert hashlib.sha256(data).hexdigest() == manifest['sha256'] == '87c3902373d3a5e9e27c09522ad9adc5f4c6e6822eb2f64aa613d82bc1638c98'
assert (manifest['part_id'], manifest['quads'], manifest['nodes']) == (2000524, 4250, 4384)
assert manifest['thickness_m'] == .0005
assert manifest['complete_part_shells'] == 4251
assert [v['element_id'] for v in manifest['excluded_from_quad_fixture']] == [2357656]
for block in manifest['source_blocks'].values():
    assert hashlib.sha256(block['raw_text'].encode()).hexdigest() == block['sha256']
print('Original PID2000524 quad fixture and complete material/section/source identity verified')
