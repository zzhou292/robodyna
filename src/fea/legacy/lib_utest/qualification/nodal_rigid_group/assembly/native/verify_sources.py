#!/usr/bin/env python3
"""Authenticate complete donor bytes and exact native arithmetic extractions."""
import hashlib
import json
from pathlib import Path

root=Path(__file__).resolve().parent
manifest=json.loads((root/'source-manifest.json').read_text())
for source in manifest['sources']:
    raw=(root/source['path']).read_bytes()
    assert len(raw)==source['bytes'] and hashlib.sha256(raw).hexdigest()==source['sha256'],source['path']
    assert hashlib.sha1(b'blob '+str(len(raw)).encode()+b'\0'+raw).hexdigest()==source['git_blob_sha1'],source['path']
for fragment in manifest['fragments']:
    lines=(root/fragment['source']).read_bytes().splitlines(keepends=True)
    raw=b''.join(lines[fragment['first_line']-1:fragment['last_line']])
    assert raw==(root/fragment['path']).read_bytes(),fragment['path']
    assert hashlib.sha256(raw).hexdigest()==fragment['sha256'],fragment['path']
print(f"Verified {len(manifest['sources'])} pinned sources and {len(manifest['fragments'])} exact fragments")
