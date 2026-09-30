#!/usr/bin/env python3
"""Check the frozen original-source fixture and its complete card receipt."""
import hashlib
import json
from pathlib import Path

root = Path(__file__).resolve().parent / 'source_fixture'
manifest_bytes = (root / 'source-manifest.json').read_bytes()
assert hashlib.sha256(manifest_bytes).hexdigest() == '82da404daada1c5c607cf374379a64a4027e8df50094e1ae42f432a9d996c9d1'
manifest = json.loads(manifest_bytes)
assert manifest['archive_sha256'] == 'aff8194c456726a678d6cc11f644316ca70f3d9b37c4db622726b7b2985b0451'
assert manifest['member_sha256'] == '67208317e6c8eb1dd43b80001508915ccaace7bc0a745e1aa5a3b33f394df301'
assert manifest['canonical_manifest_sha256'] == 'c82f1886b8935d69ff7db4c29c700370e3a057579fab80d02664a253bc7af1c8'
assert (manifest['part_id'],manifest['nodes'],manifest['solids']) == (2000977,3672,908)
header = manifest['header']
assert header['sha256'] == '949fe1b0bdaf628fcefbe3f45d6f99a88d388eb793309ad828cc6200c6943878'
value = (root / header['file']).read_bytes()
assert len(value) == header['bytes'] and hashlib.sha256(value).hexdigest() == header['sha256']
for block in manifest['material_receipt']['blocks']:
    assert hashlib.sha256(block['raw_text'].encode()).hexdigest() == block['sha256']
print('Original solid18 908-cell ordered fixture identity PASS')
