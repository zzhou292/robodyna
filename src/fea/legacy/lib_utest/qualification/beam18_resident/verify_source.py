#!/usr/bin/env python3
"""Authenticate the mapped beam participant and its qualified value owners."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
manifest = json.loads((HERE / 'source-manifest.json').read_text())
for name, expected in manifest['files'].items():
    path = Path(name)
    if path.is_absolute() or '..' in path.parts:
        raise ValueError('Invalid source identity path')
    data = (ROOT / path).read_bytes()
    if len(data) != expected['bytes'] or hashlib.sha256(data).hexdigest() != expected['sha256']:
        raise ValueError('Beam resident identity changed: ' + name)
for owner in ('beam18_force', 'beam18_model'):
    subprocess.run([sys.executable, '-B', str(HERE.parent / owner / 'verify_source.py')], check=True)
print('Beam resident, endpoint helpers and existing native value/model owners authenticated')
