"""Authenticate the complete donor bytes and copy exact STI scaling loops."""
import argparse
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parent


def prepare(output, check):
    manifest = json.loads((ROOT/'source-manifest.json').read_text())
    for record in manifest['sources']:
        data = (ROOT/record['path']).read_bytes()
        blob = hashlib.sha1(b'blob '+str(len(data)).encode()+b'\0'+data).hexdigest()
        if (len(data) != record['bytes'] or hashlib.sha256(data).hexdigest() != record['sha256']
                or blob != record['git_blob_sha1']):
            raise RuntimeError('Native source changed: '+record['source'])
        first, last = record['lines']
        fragment = b''.join(data.splitlines(keepends=True)[first-1:last])
        path = output/record['include']
        if check:
            if not path.is_file() or path.read_bytes() != fragment:
                raise RuntimeError('Prepared native scaling changed')
        else:
            output.mkdir(parents=True, exist_ok=True)
            path.write_bytes(fragment)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--check', action='store_true')
    args = parser.parse_args()
    prepare(args.output, args.check)
