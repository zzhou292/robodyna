#!/usr/bin/env python3
"""Authenticate cached complete CZFORC3 elastic dispatch; other owners verify their own leaves."""
import hashlib,json
from pathlib import Path
root=Path(__file__).resolve().parent
m=json.loads((root/'source-manifest.json').read_text())
assert m['revision']=='a62b27e6baa555d222a580d6218867d0be4d70b5'
for e in m['sources']:
    b=(root/e['path']).read_bytes()
    if len(b)!=e['bytes'] or hashlib.sha256(b).hexdigest()!=e['sha256']:
        raise RuntimeError('Changed LAW1 force context donor: '+e['path'])
    if hashlib.sha1(b'blob '+str(len(b)).encode()+b'\0'+b).hexdigest()!=e['git_blob']:
        raise RuntimeError('Changed LAW1 force pinned Git identity: '+e['path'])
