#!/usr/bin/env python3
"""Authenticate the pinned reader/CFG closure; stage build-owned byte copies."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil

READER_SHA = '3d25e5306783cdfea39be4639263cd400ab8a93e34a640be4fe30a6d92ad5882'
EXTERNAL_SHA = '57c659b803fde48c366aa025d5284c907d4ec352a161180cb16712134d82f248'
PIN = 'a62b27e6baa555d222a580d6218867d0be4d70b5'
UNITS_SHA = 'a46e7d5432a1cfdd6d3d6836e25acd73b1b1f04c990fa6c5dac864cb4a026a28'


def checked(path, digest):
    b = path.read_bytes()
    if hashlib.sha256(b).hexdigest() != digest:
        raise ValueError(f'identity mismatch: {path}')
    return b


def records(cache, filename, digest, source):
    m = json.loads(checked(cache / filename, digest))
    if filename.startswith('reader') and m['revision'] != PIN:
        raise ValueError('reader revision')
    for r in m['files']:
        relative = Path(r['source'])
        if relative.is_absolute() or '..' in relative.parts:
            raise ValueError('unsafe source path')
        b = checked(cache / source / relative, r['sha256'])
        if len(b) != r['bytes']:
            raise ValueError('source byte count')
        yield relative, b


def stage(cache, units, output, check=False):
    count = total = 0
    for filename, digest, source in (
        ('reader-source-manifest.json', READER_SHA, 'original'),
        ('external-source-manifest.json', EXTERNAL_SHA, 'external')):
        for relative, b in records(cache, filename, digest, source):
            count += 1
            total += len(b)
            if count > 3130 or total > 42000000:
                raise ValueError('source closure bound')
            if output and not check:
                p = output / relative
                p.parent.mkdir(parents=True, exist_ok=True)
                if not p.exists() or p.read_bytes() != b:
                    p.write_bytes(b)
    lines = checked(units, UNITS_SHA).decode().splitlines(True)
    region = ''.join(lines[508:513])
    if 'FAC_M ** MASS_DIM' not in region or 'RVAL  = DVAL *  FAC' not in region:
        raise ValueError('native unit region moved')
    if output and not check:
        (output / 'units_values.inc').write_text(region)
    print(f'authenticated {count} files / {total} bytes; full HM_GET_FLOATV +509:513')


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('--cache', type=Path, required=True)
    p.add_argument('--units', type=Path, required=True)
    p.add_argument('--output', type=Path)
    p.add_argument('--check', action='store_true')
    a = p.parse_args()
    stage(a.cache, a.units, a.output, a.check)
