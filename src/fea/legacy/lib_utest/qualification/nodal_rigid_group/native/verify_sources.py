"""Verify retained original files and exact arithmetic extraction boundaries."""
from pathlib import Path
import hashlib
import json


def verify(root):
    manifest = json.loads((root / 'source-manifest.json').read_text())
    if manifest['commit'] != 'a62b27e6baa555d222a580d6218867d0be4d70b5':
        raise ValueError('Unexpected nodal-rigid source commit')
    originals = {}
    for source in manifest['sources']:
        data = (root / source['path']).read_bytes()
        blob = b'blob ' + str(len(data)).encode() + b'\0' + data
        if (len(data) != source['size'] or
                hashlib.sha256(data).hexdigest() != source['sha256'] or
                hashlib.sha1(blob).hexdigest() != source['git_blob_sha1']):
            raise ValueError('Pinned source changed: ' + source['path'])
        originals[source['path']] = data.splitlines(keepends=True)
    for fragment in manifest['fragments']:
        expected = b''.join(originals[fragment['source']][fragment['first_line'] - 1:fragment['last_line']])
        actual = (root / fragment['path']).read_bytes()
        if actual != expected or hashlib.sha256(actual).hexdigest() != fragment['sha256']:
            raise ValueError('Native arithmetic fragment changed: ' + fragment['path'])


if __name__ == '__main__':
    verify(Path(__file__).resolve().parent)
    print('Pinned nodal-rigid sources and exact arithmetic fragments verified')
