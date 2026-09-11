#!/usr/bin/env python3
"""Check owned adapter bytes and complete original native source identities."""
import hashlib
import importlib.util
import json
from pathlib import Path
import tempfile

ROOT = Path(__file__).resolve().parent
TL_ROOT = ROOT.parents[2]
manifest = json.loads((ROOT / 'owning-source-manifest.json').read_text())
for entry in manifest['files']:
    data = (TL_ROOT / entry['path']).read_bytes()
    if len(data) != entry['bytes'] or hashlib.sha256(data).hexdigest() != entry['sha256']:
        raise RuntimeError('Changed owned spectral source: ' + entry['path'])
spec = importlib.util.spec_from_file_location('native_spectrum_sources', ROOT / 'native/prepare_sources.py')
native = importlib.util.module_from_spec(spec)
spec.loader.exec_module(native)
with tempfile.TemporaryDirectory(prefix='native-spectrum-source-') as directory:
    native.prepare(Path(directory), False)
    native.prepare(Path(directory), True)
print('PASS: owned adapter and complete pinned native spectral/projection sources')
