#!/usr/bin/env python3
import hashlib
import json
from pathlib import Path
root = Path(__file__).resolve().parent
repo = root.parents[2]
for name, expected in json.loads((root/'source-manifest.json').read_text())['files'].items():
    data = (repo/name).read_bytes()
    if len(data) != expected['bytes'] or hashlib.sha256(data).hexdigest() != expected['sha256']:
        raise ValueError('Shared solid caller source changed: '+name)
print('Shared solid caller extraction identity authenticated')
