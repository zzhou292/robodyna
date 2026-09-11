#!/usr/bin/env python3
"""Verify complete pinned leaves, namespace them, and retain selected exact SRCOOR3 statements.

The sole instrumentation copies SDERI3B's already-computed Gauss values into
private observation storage. No native expression, array bound, or branch changes.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re

ROOT = Path(__file__).resolve().parent
MANIFEST = json.loads((ROOT / 'source-manifest.json').read_text())
SRCOOR = 'starter/source/elements/solid/solide/srcoor3.F'
SLICES = {
    'connectivity.inc': (106, 137),
    'gather.inc': (143, 168),
    'double_coordinates.inc': (249, 274),
    'frame.inc': (282, 300),
    'project.inc': (304, 383),
}
NAMES = {'constant_mod', 'precision_mod', 'element_mod', 'ale_mod', 'message_mod',
         'checkvolume_8n', 'checkvolume_6n', 'checkvolume_4n', 'srepiso3', 'sortho3',
         'basisf', 'basis8', 'sderi3b', 'smass3b'}


def prepare(output, check):
    license_record = MANIFEST['license']
    license_bytes = (ROOT / license_record['path']).read_bytes()
    if (len(license_bytes) != license_record['bytes'] or
            hashlib.sha256(license_bytes).hexdigest() != license_record['sha256']):
        raise RuntimeError('Native license changed')
    data = {}
    for entry in MANIFEST['sources']:
        value = (ROOT / entry['path']).read_bytes()
        blob = hashlib.sha1(b'blob ' + str(len(value)).encode() + b'\0' + value).hexdigest()
        if (len(value) != entry['bytes'] or hashlib.sha256(value).hexdigest() != entry['sha256']
                or blob != entry['git_blob_sha1']):
            raise RuntimeError('Native donor changed: ' + entry['path'])
        data[entry['source']] = value.decode('latin1')
    for value in data.values():
        NAMES.update(n.lower() for n in re.findall(r'COMMON\s*/\s*(\w+)\s*/', value, re.I))
    names = re.compile(r'\b(?:' + '|'.join(sorted(NAMES, key=len, reverse=True)) + r')\b', re.I)
    outputs = {Path(source).name: value for source, value in data.items() if source != SRCOOR}
    for name, (first, last) in SLICES.items():
        outputs[name] = ''.join(data[SRCOOR].splitlines(keepends=True)[first-1:last])
    value = outputs['sderi3b.F']
    use = '      USE MESSAGE_MOD\n'
    assert value.count(use) == 1
    value = value.replace(use, use + '      USE SOLID18_POINT_CAPTURE, ONLY: POINT_VALUES\n')
    anchor = '        DO I=1,NEL\n          DET(I)=DET(I)+VLINC(I,JPT)'
    assert value.count(anchor) == 1
    capture = 'C     Test-only observation of native values; selected wrapper has NEL=1.\n'
    capture += '        POINT_VALUES(1:8,JPT)=H(1:8)\n'
    for node in range(1, 9):
        for component, axis in enumerate('XYZ'):
            slot = 9 + 3*(node-1) + component
            capture += f'        POINT_VALUES({slot},JPT)=P{axis}{node}(1)\n'
    capture += '        POINT_VALUES(33,JPT)=VLINC(1,JPT)\n'
    outputs['sderi3b.F'] = value.replace(anchor, capture + anchor)
    for name, value in outputs.items():
        result = names.sub(lambda m: 'SOLID18_REF_' + m.group().upper(), value).encode('latin1')
        path = output / name
        if check:
            if not path.is_file() or path.read_bytes() != result:
                raise RuntimeError('Prepared donor changed: ' + name)
        else:
            output.mkdir(parents=True, exist_ok=True)
            path.write_bytes(result)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--check', action='store_true')
    args = parser.parse_args()
    prepare(args.output, args.check)
