#!/usr/bin/env python3
"""Authenticated complete leaves and exact, selected caller/setup slices."""
import argparse
import hashlib
import json
from pathlib import Path
import re

ROOT = Path(__file__).resolve().parent
SHARED = ROOT / '../../native/qeph/original'
MANIFEST = json.loads((ROOT / 'source-manifest.json').read_text())
SLICES = {
    'setup.inc': ('starter/source/materials/mat/mat036/hm_read_mat36.F', 267, 318),
    'sentinels.inc': ('starter/source/materials/mat/mat036/hm_read_mat36.F', 228, 253),
    'increments.inc': ('engine/source/materials/mat_share/mulaw.F90', 718, 734),
    'strains.inc': ('engine/source/materials/mat_share/mulaw.F90', 885, 898),
    'plastic_work.inc': ('engine/source/materials/mat_share/mulaw.F90', 2204, 2217),
    'internal_work.inc': ('engine/source/materials/mat_share/mulaw.F90', 2996, 3010),
    'density.inc': ('engine/source/materials/mat_share/mmain.F90', 693, 693),
    'average_volume.inc': ('engine/source/materials/mat_share/mmain.F90', 683, 683),
    'stored_energy.inc': ('engine/source/materials/mat_share/mmain.F90', 1997, 2003),
}
NAMES = {'constant_mod', 'precision_mod', 'sigeps36', 'm36iter_imp',
         'mstrain_rate', 'vinter', 'vinter2', 'vinter2dp', 'imp_stop'}

def prepare(output, check):
    data = {}
    for entry in MANIFEST['sources']:
        value = (ROOT / entry['path']).read_bytes()
        if len(value) != entry['bytes'] or hashlib.sha256(value).hexdigest() != entry['sha256']:
            raise RuntimeError('Native donor changed: ' + entry['path'])
        blob = hashlib.sha1(b'blob ' + str(len(value)).encode() + b'\0' + value).hexdigest()
        if blob != entry['git_blob_sha1']:
            raise RuntimeError('Native Git blob changed: ' + entry['path'])
        data[entry['source']] = value.decode('latin1')
    for value in data.values():
        NAMES.update(n.lower() for n in re.findall(r'COMMON\s*/\s*(\w+)\s*/', value, re.I))
    tokens = re.compile(r'\b(?:' + '|'.join(sorted(NAMES, key=len, reverse=True)) + r')\b', re.I)
    outputs = {}
    for source, value in data.items():
        if source.endswith('/sigeps36.F') or source.endswith('/m36iter_imp.F') or source.endswith('/mstrain_rate.F') or source.endswith('/vinter.F'):
            outputs[Path(source).name] = value
        if '/share/' in source or '/modules/constant_mod.' in source or '/modules/precision_mod.' in source:
            outputs[Path(source).name] = value
    for name, (source, first, last) in SLICES.items():
        outputs[name] = ''.join(data[source].splitlines(keepends=True)[first-1:last])
    # Private namespaces only: no material expressions, array bounds or branches change.
    for name, value in outputs.items():
        result = tokens.sub(lambda m: 'LAW36_REF_' + m.group().upper(), value).encode('latin1')
        path = output / name
        if check:
            if not path.is_file() or path.read_bytes() != result:
                raise RuntimeError('Prepared source changed: ' + name)
        else:
            output.mkdir(parents=True, exist_ok=True)
            path.write_bytes(result)

if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--check', action='store_true')
    a = p.parse_args()
    prepare(a.output, a.check)
