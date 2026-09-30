#!/usr/bin/env python3
import hashlib
import json
from pathlib import Path
root=Path(__file__).resolve().parent/'source_fixture'
m=json.loads((root/'source-manifest.json').read_text())
assert m['archive_sha256']=='aff8194c456726a678d6cc11f644316ca70f3d9b37c4db622726b7b2985b0451'
assert m['member_sha256']=='67208317e6c8eb1dd43b80001508915ccaace7bc0a745e1aa5a3b33f394df301'
for r in m['files']:
    b=(root/r['path']).read_bytes()
    assert len(b)==r['bytes'] and hashlib.sha256(b).hexdigest()==r['sha256']
r=json.loads((root/'source-receipt.json').read_text())
for b in r['blocks']:
    assert hashlib.sha256(b['raw_text'].encode()).hexdigest()==b['sha256']
assert [b['first_line'] for b in r['blocks']]==[18003,18008,18011,18183]
print('LAW36 original complete card/curve receipt PASS')
