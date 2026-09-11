#!/usr/bin/env python3
"""Use authenticated source blocks verbatim; only labeled controls alter fields."""
import argparse
import hashlib
import json
from pathlib import Path

MANIFEST_SHA = 'c5adae0a61384bb7b57c88ed113798cf5b0fbaa8758fd48f93e15cc61f65f94a'
SOURCE_SHA = '67208317e6c8eb1dd43b80001508915ccaace7bc0a745e1aa5a3b33f394df301'


def fixture(receipt, output):
    raw = receipt.read_bytes()
    if hashlib.sha256(raw).hexdigest() != MANIFEST_SHA:
        raise ValueError('original receipt identity')
    m = json.loads(raw)
    if m['source_length_unit'] != 'mm' or m['source_mass_unit'] != 't':
        raise ValueError('source unit identity')
    if m['length_scale'] != .001 or m['density_to_si'] != 1e12:
        raise ValueError('canonical unit factors')
    blocks = m['declarations']['blocks']
    selected = []
    for family, identity in [('part', 2000063), ('section', 2000063),
                             ('material', 2000063), ('curve', 2100015)]:
        rows = [b for b in blocks if b['family'] == family and b['identity'] == identity]
        if len(rows) != 1:
            raise ValueError('selected block identity')
        b = rows[0]
        text = b['raw_text']
        if hashlib.sha256(text.encode()).hexdigest() != b['sha256']:
            raise ValueError('source block digest')
        selected.append(b)
    # This wrapper is an explicit native test unit context, not a source card.
    wrapper = '*KEYWORD\n*CONTROL_UNITS\n' + ''.join(f'{s:<10}' for s in ('mm', 'sec', 'mtrc_ton', 'K')) + '\n'
    body = ''.join(b['raw_text'] + ('' if b['raw_text'].endswith('\n') else '\n') for b in selected)
    baseline = wrapper + body + '*END\n'
    material = next(b for b in selected if b['family'] == 'material')
    first_card = material['cards'][0]['text']
    variants = {'original': baseline}
    for name, field, value in [('explicit_hu_one', 5, '1.0'), ('source_damp_control', 7, '0.2')]:
        padded = first_card.ljust(80)
        changed = padded[:10*field] + f'{value:>10}' + padded[10*(field+1):]
        variants[name] = baseline.replace(first_card, changed, 1)
    output.mkdir(parents=True, exist_ok=True)
    records = {}
    for name, text in variants.items():
        p = output / (name + '.key')
        if p.exists() and p.read_text() != text:
            raise ValueError('existing fixture differs')
        p.write_text(text)
        records[name] = {'file': p.name, 'bytes': len(text.encode()),
                         'sha256': hashlib.sha256(text.encode()).hexdigest()}
    curve = next(b for b in selected if b['family'] == 'curve')
    xy = [float(v) for card in curve['cards'][1:] for v in card['text'].split()]
    if len(xy) != 56:
        raise ValueError('original curve shape')
    manifest = {'schema': 'law90.sdi_original_fixture.v1', 'receipt_sha256': MANIFEST_SHA,
                'source_sha256': SOURCE_SHA, 'source_units': 'mm/s/tonne',
                'wrapper_is_explicit_test_context': True, 'files': records,
                'curve_working_xy': xy, 'source_blocks': selected}
    (output / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    return manifest


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('--receipt', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    fixture(a.receipt, a.output)
