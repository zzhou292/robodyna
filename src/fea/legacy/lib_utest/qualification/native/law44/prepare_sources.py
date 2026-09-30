#!/usr/bin/env python3
"""Verify exact native donors and prepare private symbol names only."""
import argparse
import hashlib
import json
from pathlib import Path
import re

ROOT = Path(__file__).resolve().parent
NAMES = ('constant_mod', 'precision_mod', 'sigeps44c', 'vinter2dp', 'vinter2', 'vinter',
         'coqini_wm', 'coqini', 'com20', 'param', 'parit')
TOKEN = re.compile(r'\b(?:' + '|'.join(NAMES) + r')\b', re.IGNORECASE)
PREPARE = {
    'original/sigeps44c.F': 'Sigeps44c.F',
    'original/vinter.F': 'Vinter.F',
    'original/coqini.F': 'Coqini.F',
    'original/com20_c.inc': 'com20_c.inc',
    '../qeph/original/common_source/modules/constant_mod.F': 'Constant.F',
    '../qeph/original/common_source/modules/precision_mod.F90': 'Precision.F90',
    '../qeph/original/engine/share/spe_inc/implicit_f.inc': 'implicit_f.inc',
    '../qeph/original/engine/share/includes/param_c.inc': 'param_c.inc',
}


def prepare(output, check):
    manifest = json.loads((ROOT / 'source-manifest.json').read_text())
    for entry in manifest['sources']:
        data = (ROOT / entry['path']).read_bytes()
        if len(data) != entry['bytes'] or hashlib.sha256(data).hexdigest() != entry['sha256']:
            raise RuntimeError('Native source changed: ' + entry['path'])
    for source, destination in PREPARE.items():
        data = (ROOT / source).read_bytes()
        result = TOKEN.sub(lambda match: 'LAW44_POINT_REF_' + match.group().upper(),
                           data.decode('latin1')).encode('latin1')
        path = output / destination
        if check:
            if not path.is_file() or path.read_bytes() != result:
                raise RuntimeError('Prepared native source changed: ' + destination)
        else:
            output.mkdir(parents=True, exist_ok=True)
            if not path.is_file() or path.read_bytes() != result:
                path.write_bytes(result)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--check', action='store_true')
    args = parser.parse_args()
    prepare(args.output, args.check)
