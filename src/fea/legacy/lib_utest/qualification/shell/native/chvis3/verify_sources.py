#!/usr/bin/env python3
"""Read-only verification of the pinned original CHVIS3 dependency closure."""
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parent / 'original'
MANIFEST_SHA = '853aaaa971281d4caf2d3e163ffd2780800332308dfe7d974a4a4847070f4c4c'


def verify():
    raw = (ROOT / 'source-manifest.json').read_bytes()
    if hashlib.sha256(raw).hexdigest() != MANIFEST_SHA:
        raise ValueError('Original dependency manifest changed; audit required')
    manifest = json.loads(raw)
    if manifest['revision'] != 'a62b27e6baa555d222a580d6218867d0be4d70b5':
        raise ValueError('Original dependency revision changed')
    for entry in manifest['files']:
        data = (ROOT / entry['file']).read_bytes()
        blob = hashlib.sha1(b'blob ' + str(len(data)).encode() + b'\0' + data).hexdigest()
        if (len(data) != entry['bytes'] or hashlib.sha256(data).hexdigest() != entry['sha256']
                or blob != entry['git_blob_sha1']):
            raise ValueError('Original dependency differs: ' + entry['file'])
    return dict(status='verified', original_files=len(manifest['files']),
                total_bytes=manifest['total_bytes'], manifest_sha256=MANIFEST_SHA)


if __name__ == '__main__':
    print(json.dumps(verify(), indent=2))
