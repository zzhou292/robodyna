#!/usr/bin/env python3
"""Read-only ownership receipt; composing native owners verify full donor trees."""
import hashlib
import json
from pathlib import Path
MANIFEST_SHA256 = "02591fbdee1fabb6f979e6aa5fbb8d936fd20b0f2adf81f4d16c1178788abbf5"
def verify():
    directory=Path(__file__).resolve().parent
    root=directory.parents[2]
    raw=(directory/'source-manifest.json').read_bytes()
    if hashlib.sha256(raw).hexdigest()!=MANIFEST_SHA256:
        raise RuntimeError('Mapped Q/T ownership receipt changed')
    manifest=json.loads(raw)
    for record in manifest['files']:
        path=Path(record['path'])
        if path.is_absolute() or '..' in path.parts:
            raise RuntimeError('Unsafe ownership path')
        data=(root/path).read_bytes()
        if len(data)!=record['bytes'] or hashlib.sha256(data).hexdigest()!=record['sha256']:
            raise RuntimeError('Mapped Q/T source changed: '+str(path))
    return {'status':'passed','records':len(manifest['files']),'donor_revision':manifest['donor_revision'],'numerical_execution':False}
if __name__=='__main__':
    print(json.dumps(verify(),sort_keys=True))
