#!/usr/bin/env python3
"""Check the owned material fixture and complete pinned native preparation."""
import hashlib
import importlib.util
import json
from pathlib import Path
import tempfile

ROOT = Path(__file__).resolve().parent
TL_ROOT = ROOT.parents[2]
manifest = json.loads((ROOT / 'owning-source-manifest.json').read_text())
for entry in manifest['files']:
    path = TL_ROOT / entry['path']
    data = path.read_bytes()
    if len(data) != entry['bytes'] or hashlib.sha256(data).hexdigest() != entry['sha256']:
        raise RuntimeError('Changed owned source: ' + entry['path'])

receipt = json.loads((ROOT / 'source_fixture/source-receipt.json').read_text())
if receipt['part_id'] != 2000063 or receipt['curve_id'] != 2100015:
    raise RuntimeError('Wrong original material identity')
curve = next(row for row in receipt['source_blocks'] if row['family'] == 'curve')
points = [[float(value) for value in row['raw'].split()] for row in curve['cards'][1:]]
if points != receipt['curve_points'] or len(points) != 28:
    raise RuntimeError('Raw original curve differs from fixture')
header = '// Authenticated original PID/MID2000063, curve2100015. See source-receipt.json.\n#pragma once\n#include <cstdint>\nnamespace original_radiator {\ninline constexpr std::uint32_t point_count = 28;\n'
for name, values in [('strain', [p[0] for p in points]),
                     ('stress_mpa', [p[1] for p in points]),
                     ('stress_pa', [p[1] * 1e6 for p in points])]:
    header += 'inline constexpr double ' + name + '[] = {\n'
    header += ''.join('  ' + value.hex() + ',\n' for value in values) + '};\n'
header += 'inline constexpr double density_kg_m3 = ' + (7.7200e-10 * 1e12).hex() + ';\n}\n'
if (ROOT / 'source_fixture/OriginalMaterial.h').read_text() != header:
    raise RuntimeError('Original binary64 fixture differs')

spec = importlib.util.spec_from_file_location('native_preparation', ROOT / 'native/prepare_sources.py')
native = importlib.util.module_from_spec(spec)
spec.loader.exec_module(native)
with tempfile.TemporaryDirectory(prefix='law90-source-') as directory:
    native.prepare(Path(directory), False)
    native.prepare(Path(directory), True)
print('PASS: original28-point receipt, owned sources and complete native donor identities')
