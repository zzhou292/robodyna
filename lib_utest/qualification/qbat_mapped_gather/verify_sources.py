#!/usr/bin/env python3
"""Authenticate complete frozen serial sources and unchanged numerical leaves."""
from pathlib import Path
import hashlib,json
here=Path(__file__).resolve().parent
root=here.parents[2]
raw=(here/'source-manifest.json').read_bytes()
assert hashlib.sha256(raw).hexdigest()=="7a4293e04acb72d315c5a751f1f07609a6294f3cea5ebd4d91e4b59b34ae43a1"
manifest=json.loads(raw)
for row in manifest['files']:
    path=Path(row['path'])
    assert not path.is_absolute() and '..' not in path.parts
    value=(root/path).read_bytes()
    assert len(value)==row['bytes'] and hashlib.sha256(value).hexdigest()==row['sha256'],path
for row in manifest['baselines']:
    value=(here/row['fixture']).read_text()
    for old,new in reversed(row['wrappers']):
        assert new in value,(row['fixture'],new)
        value=value.replace(new,old)
    raw=value.encode()
    assert len(raw)==row['bytes'] and hashlib.sha256(raw).hexdigest()==row['sha256'],row['path']
print(json.dumps({'status':'passed','records':len(manifest['files']),
    'frozen_complete_sources':len(manifest['baselines']),
    'baseline':manifest['baseline_commit'],'numerical_execution':False}))
