#!/usr/bin/env python3
"""Authenticate complete donors and extract the complete native VALPVEC_V."""
import argparse
import hashlib
import json
from pathlib import Path
import re

ROOT = Path(__file__).resolve().parent
NAMES = {'constant_mod', 'precision_mod', 'valpvec_v', 'floatmin', 'floatmin_',
         'floatmin__', 'my_shiftl', 'my_shiftl_', 'my_shiftr', 'my_shiftr_',
         'my_and', 'my_and_', 'my_or', 'my_or_'}


def prepare(output, check):
    result = {}
    for entry in json.loads((ROOT / 'source-manifest.json').read_text())['sources']:
        raw = (ROOT / entry['path']).read_bytes()
        blob = hashlib.sha1(b'blob ' + str(len(raw)).encode() + b'\0' + raw).hexdigest()
        if (len(raw) != entry['bytes'] or hashlib.sha256(raw).hexdigest() != entry['sha256']
                or blob != entry['git_blob_sha1']):
            raise RuntimeError('Complete native donor changed: ' + entry['source'])
        value = raw.decode('latin1')
        leaf = Path(entry['source']).name
        if leaf == 'sigeps33.F':
            match = re.search(r'^      SUBROUTINE VALPVEC_V\(.*?^      END\s*$',
                              value, re.M | re.S)
            if not match:
                raise RuntimeError('Complete VALPVEC_V missing')
            result['VALPVEC_V.F'] = match.group() + '\n'
        elif leaf == 'sigeps90.F':
            # Exact selected projection expressions; wrappers bind all scalars.
            result['projection.inc'] = ''.join(value.splitlines(keepends=True)[293:318])
        else:
            result[leaf] = value
    tokens = re.compile(r'\b(?:' + '|'.join(sorted(NAMES, key=len, reverse=True)) + r')\b', re.I)
    for name, value in result.items():
        # Preserve C case: the complete file intentionally has upper/lower APIs.
        def rename(match):
            if name.endswith('.c'):
                return 'spectrum_ref_' + match.group()
            return 'SPECTRUM_REF_' + match.group().upper()
        raw = tokens.sub(rename, value).encode('latin1')
        path = output / name
        if check:
            if not path.is_file() or path.read_bytes() != raw:
                raise RuntimeError('Prepared native source changed: ' + name)
        else:
            output.mkdir(parents=True, exist_ok=True)
            path.write_bytes(raw)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--check', action='store_true')
    args = parser.parse_args()
    prepare(args.output, args.check)
