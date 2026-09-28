#!/usr/bin/env python3
from pathlib import Path
import hashlib,json,subprocess
here=Path(__file__).resolve().parent
root=here.parents[2]
manifest=json.loads((here/'source-manifest.json').read_text())
for name,record in manifest['files'].items():
    data=(root/name).read_bytes()
    assert len(data)==record['bytes'] and hashlib.sha256(data).hexdigest()==record['sha256'],name
changed=subprocess.check_output(['git','diff','--name-only',manifest['base'],'--','lib_src'],cwd=root,text=True).splitlines()
assert set(changed)==set(manifest['production_changes']),changed
print('Pinned',len(manifest['files']),'files; only the reviewed CIN read-scheduling production delta is present')
