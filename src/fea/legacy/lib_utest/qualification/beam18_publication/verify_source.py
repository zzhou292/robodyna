#!/usr/bin/env python3
"""Check test-owned sources; shared fixtures retain their existing owning gates."""
import hashlib
import json
from pathlib import Path

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[2]
manifest=json.loads((HERE/'source-manifest.json').read_text())
for name,expected in manifest['files'].items():
    path=Path(name)
    if path.is_absolute() or '..' in path.parts:
        raise ValueError('Invalid qualifier source path')
    data=(ROOT/path).read_bytes()
    if len(data)!=expected['bytes'] or hashlib.sha256(data).hexdigest()!=expected['sha256']:
        raise ValueError('Common beam publication qualifier identity changed: '+name)
print('Beam common-publication test identities PASS; shared fixture/production/native gates remain separately owned')
