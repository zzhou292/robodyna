"""Reuse authenticated donor backing and generate exact complete routines."""
from pathlib import Path
import argparse
import hashlib
import importlib.util
import json


def stage(tl_root, cache, destination):
    own = Path(__file__).resolve().parent
    manifest = json.loads((own / 'source-manifest.json').read_text())
    resolved = {'commit': manifest['commit'], 'sources': [], 'fragments': []}
    pieces = []
    for item in manifest['sources']:
        root = {'tl': tl_root, 'cache': cache}[item['owner']]
        source = root / item['path']
        data = source.read_bytes()
        blob = b'blob ' + str(len(data)).encode() + b'\0' + data
        if (len(data) != item['size'] or hashlib.sha256(data).hexdigest() != item['sha256']
                or hashlib.sha1(blob).hexdigest() != item['git_blob_sha1']):
            raise ValueError('Pinned CIN source changed: ' + str(source))
        exact = b''.join(data.splitlines(keepends=True)[item['first_line']-1:item['last_line']])
        if hashlib.sha256(exact).hexdigest() != item['fragment_sha256']:
            raise ValueError('Pinned CIN extraction changed: ' + item['output'])
        pieces.append((item['output'], exact))
        resolved['sources'].append({key: item[key] for key in ('size', 'sha256', 'git_blob_sha1')}
                                   | {'path': str(source)})
        resolved['fragments'].append({'path': item['output'], 'source': str(source),
            'first_line': item['first_line'], 'last_line': item['last_line'],
            'sha256': item['fragment_sha256']})
    destination.mkdir(parents=True, exist_ok=True)
    for name, data in pieces:
        path = destination / name
        if not path.exists() or path.read_bytes() != data:
            path.write_bytes(data)
    (destination / 'source-manifest.json').write_text(json.dumps(resolved, indent=2) + '\n')
    shared = tl_root / 'lib_utest/qualification/nodal_rigid_group/native/verify_sources.py'
    spec = importlib.util.spec_from_file_location('native_provenance', shared)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    module.verify(destination)
    print('Five original donors and complete CIN extraction boundaries verified')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('tl_root', type=Path)
    parser.add_argument('cache', type=Path)
    parser.add_argument('destination', type=Path)
    args = parser.parse_args()
    stage(args.tl_root.resolve(), args.cache.resolve(), args.destination.resolve())
