#!/usr/bin/env python3
"""Authenticate this source/ownership delta and the existing independent native owners."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys
HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[2]
manifest=json.loads((HERE/'source-manifest.json').read_text())
for name,record in manifest['files'].items():
    path=Path(name)
    if path.is_absolute() or '..' in path.parts: raise ValueError('Invalid receipt path')
    data=(ROOT/path).read_bytes()
    if len(data)!=record['bytes'] or hashlib.sha256(data).hexdigest()!=record['sha256']:
        raise ValueError('Beam model source identity changed: '+name)
for owner,script in [('beam18_force','verify_source.py'),('extended_solid_coefficients','verify_sources.py')]:
    subprocess.run([sys.executable,'-B',str(HERE.parent/owner/script)],check=True)
print('Beam model/snapshot/V5 ledger source and native dependency identities PASS; no native execution claim')
