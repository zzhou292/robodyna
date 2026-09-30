#!/usr/bin/env python3
"""Verify owning code, reused original startup, and complete native sources."""
import hashlib
import importlib.util
import json
from pathlib import Path
import runpy
import tempfile
ROOT=Path(__file__).resolve().parent
TL_ROOT=ROOT.parents[2]
m=json.loads((ROOT/'owning-source-manifest.json').read_text())
for entry in m['files']:
    data=(TL_ROOT/entry['path']).read_bytes()
    if len(data)!=entry['bytes'] or hashlib.sha256(data).hexdigest()!=entry['sha256']:
        raise ValueError('owning source changed: '+entry['path'])
runpy.run_path(str(ROOT.parent/'solid18_reference/verify_source.py'))
spec=importlib.util.spec_from_file_location('law90_element_native',ROOT/'native/prepare_sources.py')
native=importlib.util.module_from_spec(spec)
spec.loader.exec_module(native)
with tempfile.TemporaryDirectory(prefix='law90-element-native-') as directory:
    native.prepare(Path(directory),False)
    native.prepare(Path(directory),True)
print('PASS: LAW90 solid18 owning/complete native identities; no source admission')
