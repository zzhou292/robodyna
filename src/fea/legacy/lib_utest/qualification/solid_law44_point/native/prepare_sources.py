#!/usr/bin/env python3
"""Authenticate complete donors; namespace only, plus exact bounded caller slices."""
import argparse
import hashlib
import json
from pathlib import Path
import re

ROOT = Path(__file__).resolve().parent
SLICES = {
    'setup.inc': ('starter/source/materials/mat/mat044/hm_read_mat44.F', 178, 287),
    'increments.inc': ('engine/source/materials/mat_share/mulaw.F90', 718, 734),
    'strains.inc': ('engine/source/materials/mat_share/mulaw.F90', 885, 898),
    'filter.inc': ('engine/source/materials/mat_share/mulaw.F90', 1049, 1052),
    'filter_setup.inc': ('starter/source/materials/mat/hm_read_mat.F90', 1524, 1528),
    'point_interface.inc': ('engine/source/materials/mat/mat044/sigeps44.F', 33, 141),
}

def prepare(output, check):
    data = {}
    for entry in json.loads((ROOT / 'source-manifest.json').read_text())['sources']:
        value = (ROOT / entry['path']).read_bytes()
        blob = hashlib.sha1(b'blob ' + str(len(value)).encode() + b'\0' + value).hexdigest()
        if (len(value) != entry['bytes'] or blob != entry['git_blob_sha1'] or
                hashlib.sha256(value).hexdigest() != entry['sha256']):
            raise RuntimeError('Donor identity changed: ' + entry['source'])
        data[entry['source']] = value.decode('latin1')
    names = {'constant_mod', 'precision_mod', 'sigeps44', 'mstrain_rate',
             'vinter', 'vinter2', 'vinter2dp'}
    for value in data.values():
        names.update(x.lower() for x in re.findall(r'COMMON\s*/\s*(\w+)\s*/', value, re.I))
    tokens = re.compile(r'\b(?:' + '|'.join(sorted(names, key=len, reverse=True)) + r')\b', re.I)
    outputs = {}
    for source, value in data.items():
        if (Path(source).name in {'sigeps44.F', 'mstrain_rate.F', 'vinter.F',
                                 'constant_mod.F', 'precision_mod.F90'} or '/share/' in source):
            outputs[Path(source).name] = value
    for name, (source, first, last) in SLICES.items():
        outputs[name] = ''.join(data[source].splitlines(keepends=True)[first - 1:last])
    for name, value in outputs.items():
        value = tokens.sub(lambda m: 'L44S_REF_' + m.group().upper(), value).encode('latin1')
        path = output / name
        if check:
            if not path.is_file() or path.read_bytes() != value:
                raise RuntimeError('Prepared donor changed: ' + name)
        else:
            output.mkdir(parents=True, exist_ok=True)
            path.write_bytes(value)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--check', action='store_true')
    args = parser.parse_args()
    prepare(args.output, args.check)
