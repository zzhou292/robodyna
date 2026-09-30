#!/usr/bin/env python3
"""Frozen pre-parallelization assembly and unchanged shared leaf receipt."""
from pathlib import Path
import hashlib,json
here=Path(__file__).resolve().parent
root=here.parents[2]
raw=(here/'source-manifest.json').read_bytes()
assert hashlib.sha256(raw).hexdigest()=="a91ef17d1872725a2e4ca98936e93b4d1c20b2cae2e8a5e84bca478c185141ba"
manifest=json.loads(raw)
for row in manifest['files']:
    path=Path(row['path'])
    assert not path.is_absolute() and '..' not in path.parts
    value=(root/path).read_bytes()
    assert len(value)==row['bytes'] and hashlib.sha256(value).hexdigest()==row['sha256'],path
print(json.dumps({'status':'passed','records':len(manifest['files']),'baseline':manifest['baseline_commit'],'numerical_execution':False}))
