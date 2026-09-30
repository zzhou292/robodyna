#!/usr/bin/env python3
"""Verify this slice and the reused reader, curve and reference dependencies."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parent
TL = ROOT.parents[2]
manifest = json.loads((ROOT / 'source-manifest.json').read_text())
for name, entry in manifest['files'].items():
    data = (TL / name).read_bytes()
    if len(data) != entry['bytes'] or hashlib.sha256(data).hexdigest() != entry['sha256']:
        raise ValueError('Beam force source identity changed: ' + name)
with tempfile.TemporaryDirectory(prefix='beam18-force-identity-') as directory:
    command = [sys.executable, '-B', str(ROOT / 'native/prepare_sources.py'), '--output', directory]
    subprocess.run(command, check=True)
    subprocess.run(command + ['--check'], check=True)
for owner in ('beam18_reference', 'solid_law44_point'):
    subprocess.run([sys.executable, '-B', str(ROOT.parent / owner / 'verify_source.py')], check=True)
print('Beam force source, complete native, reference and LAW44 dependency identities PASS')
