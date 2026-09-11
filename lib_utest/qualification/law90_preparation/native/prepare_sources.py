#!/usr/bin/env python3
"""Authenticate full donors; retain complete leaves and exact reader regions."""
import argparse
import hashlib
import json
from pathlib import Path
import re

ROOT = Path(__file__).resolve().parent
READER = 'starter/source/materials/mat/mat090/hm_read_mat90.F'
REGIONS = {
    'reader_flags.inc': (107, 109),
    'reader_cutoffs.inc': (137, 139),
    'reader_values.inc': (150, 213),
}
NAMES = {'constant_mod', 'precision_mod', 'message_mod', 'table_mod',
         'names_and_titles_mod', 'func_slope', 'unify_x', 'law90_upd', 'vinter',
         'vinter2', 'vinter2dp', 'finter2'}


def prepare(destination, check):
    manifest = json.loads((ROOT / 'source-manifest.json').read_text())
    data = {}
    for entry in manifest['sources']:
        raw = (ROOT / entry['path']).read_bytes()
        blob = hashlib.sha1(b'blob ' + str(len(raw)).encode() + b'\0' + raw).hexdigest()
        if len(raw) != entry['bytes'] or hashlib.sha256(raw).hexdigest() != entry['sha256'] or blob != entry['git_blob_sha1']:
            raise RuntimeError('Changed complete donor: ' + entry['source'])
        data[entry['source']] = raw.decode('latin1')
    names = set(NAMES)
    for text in data.values():
        names.update(n.lower() for n in re.findall(r'COMMON\s*/\s*(\w+)\s*/', text, re.I))
    tokens = re.compile(r'\b(?:' + '|'.join(sorted(names, key=len, reverse=True)) + r')\b', re.I)
    outputs = {}
    for source, text in data.items():
        if source != READER:
            outputs[Path(source).name] = text
    for name, (first, last) in REGIONS.items():
        outputs[name] = ''.join(data[READER].splitlines(keepends=True)[first - 1:last])
    # Only private identifiers change. Complete numerical regions are unchanged.
    for name, text in outputs.items():
        value = tokens.sub(lambda m: 'LAW90_REF_' + m.group().upper(), text).encode('latin1')
        path = destination / name
        if check:
            if not path.is_file() or path.read_bytes() != value:
                raise RuntimeError('Changed prepared donor: ' + name)
        else:
            destination.mkdir(parents=True, exist_ok=True)
            path.write_bytes(value)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--check', action='store_true')
    args = parser.parse_args()
    prepare(args.output, args.check)
