#!/usr/bin/env python3
"""Authenticate this bounded profile and its reused complete native geometry."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parent
TL = ROOT.parents[2]
manifest = json.loads((ROOT/'source-manifest.json').read_text())
for name, expected in manifest['files'].items():
    path = TL/name
    data = path.read_bytes()
    if len(data) != expected['bytes'] or hashlib.sha256(data).hexdigest() != expected['sha256']:
        raise ValueError('rear reference identity changed: '+name)
with tempfile.TemporaryDirectory(prefix='rear18-native-identity-') as directory:
    generator = ROOT.parent/'solid18_reference/native/prepare_sources.py'
    command = [sys.executable, '-B', str(generator), '--output', directory]
    subprocess.run(command, check=True)
    subprocess.run(command+['--check'], check=True)
print('Rear profile, wrappers and reused complete native donors authenticated')
