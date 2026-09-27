#!/usr/bin/env python3
"""Authenticate the controlled resident and its qualified operator dependencies."""
from pathlib import Path
import hashlib
import json
here=Path(__file__).resolve().parent
root=here.parents[2]
manifest=json.loads((here/'source-manifest.json').read_text())
for row in manifest['files']:
    raw=(root/row['path']).read_bytes()
    if len(raw)!=row['bytes'] or hashlib.sha256(raw).hexdigest()!=row['sha256']:
        raise SystemExit('Changed controlled resident dependency: '+row['path'])
print('PASS: controlled resident source/operator/qualification identities:',len(manifest['files']))
