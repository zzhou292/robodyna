#!/usr/bin/env python3
"""Verify actual LAW90 force owners and all independent original source bytes."""
import hashlib,importlib.util,json,runpy,tempfile
from pathlib import Path
ROOT=Path(__file__).resolve().parent
TL=ROOT.parents[2]
for row in json.loads((ROOT/'owning-source-manifest.json').read_text())['files']:
 data=(TL/row['path']).read_bytes()
 if len(data)!=row['bytes'] or hashlib.sha256(data).hexdigest()!=row['sha256']:
  raise ValueError('LAW90 force owning source changed: '+row['path'])
for owner in ['law90_solid18_reference','law90_point']:
 runpy.run_path(str(ROOT.parent/owner/'verify_source.py'))
spec=importlib.util.spec_from_file_location('law90_force_native',ROOT/'native/prepare_sources.py')
native=importlib.util.module_from_spec(spec);spec.loader.exec_module(native)
with tempfile.TemporaryDirectory(prefix='law90-force-identity-') as directory:
 native.prepare(Path(directory),False);native.prepare(Path(directory),True)
print('PASS LAW90 force owners, complete caller sources and reused qualified identities')
