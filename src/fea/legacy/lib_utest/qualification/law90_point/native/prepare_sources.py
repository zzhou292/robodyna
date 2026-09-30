#!/usr/bin/env python3
"""Authenticate complete engine LAW90, spectral and curve donors."""
import argparse
import hashlib
import json
from pathlib import Path
import re

ROOT = Path(__file__).resolve().parent
NAMES = {'constant_mod', 'precision_mod', 'file_descriptor_mod', 'sigeps90',
         'valpvec_v', 'valpvecdp_v', 'floatmin', 'floatmin_', 'floatmin__',
         'my_shiftl', 'my_shiftl_', 'my_shiftr', 'my_shiftr_', 'my_and',
         'my_and_', 'my_or', 'my_or_', 'vinter', 'vinter2', 'vinter2dp', 'finter2'}


def prepare(output, check, observe_modulus=False):
    result = {}
    names = set(NAMES)
    for entry in json.loads((ROOT / 'source-manifest.json').read_text())['sources']:
        raw = (ROOT / entry['path']).read_bytes()
        blob = hashlib.sha1(b'blob ' + str(len(raw)).encode() + b'\0' + raw).hexdigest()
        if (len(raw) != entry['bytes'] or hashlib.sha256(raw).hexdigest() != entry['sha256']
                or blob != entry['git_blob_sha1']):
            raise RuntimeError('Complete LAW90 donor changed: ' + entry['source'])
        value = raw.decode('latin1')
        names.update(n.lower() for n in re.findall(r'COMMON\s*/\s*(\w+)\s*/', value, re.I))
        leaf = Path(entry['source']).name
        if leaf == 'sigeps33.F':
            for name in ('VALPVEC_V', 'VALPVECDP_V'):
                match = re.search(r'^      SUBROUTINE ' + name + r'\(.*?^      END\s*$',
                                  value, re.M | re.S)
                if not match:
                    raise RuntimeError('Complete native spectral routine missing: ' + name)
                result[name + '.F'] = match.group() + '\n'
        else:
            result[leaf] = value
    tokens = re.compile(r'\b(?:' + '|'.join(sorted(names, key=len, reverse=True)) + r')\b', re.I)
    for name, value in result.items():
        def rename(match):
            if name.endswith('.c'):
                return 'law90_point_ref_' + match.group()
            return 'LAW90_POINT_REF_' + match.group().upper()
        if observe_modulus and name == 'sigeps90.F':
            anchor = '            E_OLD = UVAR(II,8)'
            # Failure-enabled and FAIL0 branches have identical modulus tails.
            if value.count(anchor) != 2:
                raise RuntimeError('LAW90 modulus observation anchor changed')
            value = value.replace(anchor, anchor + '\n' +
                '            CALL LAW90_MODULUS_OBSERVE(EPST(I),YLD(I),E_OLD,E0,E_MAX)')
        raw = tokens.sub(rename, value).encode('latin1')
        path = output / name
        if check:
            if not path.is_file() or path.read_bytes() != raw:
                raise RuntimeError('Prepared LAW90 donor changed: ' + name)
        else:
            output.mkdir(parents=True, exist_ok=True)
            path.write_bytes(raw)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--check', action='store_true')
    parser.add_argument('--observe-modulus', action='store_true')
    args = parser.parse_args()
    prepare(args.output, args.check, args.observe_modulus)
