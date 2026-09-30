#!/usr/bin/env python3
"""Authenticate rear family, independent wrappers, and complete shared donors."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parent
TL = ROOT.parents[2]
manifest = json.loads((ROOT/'source-manifest.json').read_text())
for name,expected in manifest['files'].items():
    data = (TL/name).read_bytes()
    if len(data) != expected['bytes'] or hashlib.sha256(data).hexdigest() != expected['sha256']:
        raise ValueError('Rear force identity changed: '+name)
for relative in ['solid18_law44_reference','solid_law44_point']:
    subprocess.run([sys.executable,'-B',str(ROOT.parent/relative/'verify_source.py')],check=True)
with tempfile.TemporaryDirectory(prefix='rear18-force-identity-') as directory:
    for relative in ['solid18_force','solid18_law44_force']:
        out = str(Path(directory)/relative)
        command = [sys.executable,'-B',str(ROOT.parent/relative/'native/prepare_sources.py'),'--output',out]
        subprocess.run(command,check=True)
        subprocess.run(command+['--check'],check=True)
print('Rear force profile, wrapper dimensions, source geometry and complete native donors authenticated')
