#!/usr/bin/env python3
"""Verify selected point ownership and reused fixture/spectrum/native donors."""
import hashlib
import importlib.util
import json
from pathlib import Path
import runpy
import tempfile

ROOT = Path(__file__).resolve().parent
TL_ROOT = ROOT.parents[2]
manifest = json.loads((ROOT / 'owning-source-manifest.json').read_text())
for entry in manifest['files']:
    raw = (TL_ROOT / entry['path']).read_bytes()
    if len(raw) != entry['bytes'] or hashlib.sha256(raw).hexdigest() != entry['sha256']:
        raise RuntimeError('Changed LAW90 point source: ' + entry['path'])
runpy.run_path(str(ROOT.parent / 'law90_preparation/verify_source.py'))
runpy.run_path(str(ROOT.parent / 'native_symmetric_eigen3/verify_source.py'))
spec = importlib.util.spec_from_file_location('law90_point_native_sources', ROOT / 'native/prepare_sources.py')
native = importlib.util.module_from_spec(spec)
spec.loader.exec_module(native)
with tempfile.TemporaryDirectory(prefix='law90-point-source-') as directory:
    native.prepare(Path(directory), False)
    native.prepare(Path(directory), True)
print('PASS: selected point, reused source fixture and complete engine/native identities')
