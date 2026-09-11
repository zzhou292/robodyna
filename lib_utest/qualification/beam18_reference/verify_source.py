#!/usr/bin/env python3
import hashlib,json,subprocess,sys,tempfile
from pathlib import Path
ROOT=Path(__file__).resolve().parent
TL=ROOT.parents[2]
m=json.loads((ROOT/'source-manifest.json').read_text())
for name,e in m['files'].items():
    data=(TL/name).read_bytes()
    if len(data)!=e['bytes'] or hashlib.sha256(data).hexdigest()!=e['sha256']:
        raise ValueError('Beam18 source identity changed: '+name)
with tempfile.TemporaryDirectory(prefix='beam18-identity-') as directory:
    cmd=[sys.executable,'-B',str(ROOT/'native/prepare_sources.py'),'--output',directory]
    subprocess.run(cmd,check=True);subprocess.run(cmd+['--check'],check=True)
print('Beam18 selected source, wrappers and native boundaries PASS')
