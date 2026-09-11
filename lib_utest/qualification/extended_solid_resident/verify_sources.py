#!/usr/bin/env python3
"""Check exact resident integration and reuse the owning material donor gates."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys
HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
manifest = json.loads((HERE / 'source-manifest.json').read_text())
for entry in manifest['files']:
    raw = (ROOT / entry['path']).read_bytes()
    if len(raw) != entry['bytes'] or hashlib.sha256(raw).hexdigest() != entry['sha256']:
        raise RuntimeError('Changed resident source: ' + entry['path'])
for path in ('law90_preparation/verify_source.py', 'law90_solid18_force/verify_source.py',
             'solid18_law44_startup/verify_source.py'):
    subprocess.run([sys.executable, '-B', str(HERE.parent / path)], check=True)
print('PASS: extended resident integration and existing complete material/force donor identities')
