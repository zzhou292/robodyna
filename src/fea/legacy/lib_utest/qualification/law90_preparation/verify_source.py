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

observation_bytes = (ROOT / 'source_fixture/NativeSdiOriginal.json').read_bytes()
if hashlib.sha256(observation_bytes).hexdigest() != '662fba9b45aceba5da1563574f80669adca99b114f3fe918d41e1a82fd06dd88':
    raise RuntimeError('Changed executed SDI original observation')
observation = json.loads(observation_bytes)
packet = observation['direct']['native_preparation_si']
for branch in ['direct', 'exported_reread']:
    row = observation[branch]
    if (row['material_id'] != 2000063 or row['curve_id'] != 2100015 or
        row['curve_working_xy'] != [v for point in points for v in point] or
        row['native_preparation_si'] != packet or row['qualified_positive_hys_profile']):
        raise RuntimeError('Executed SDI branch/source differs')
if len(packet) != 33 or packet[16] != 1 or packet[8] != 1 or packet[11] != 1e6:
    raise RuntimeError('Wrong native pre/post default classifier or curve units')
# HM_GET_FLOATV keeps mass, length and time multiplications in that order.
density = 7.7200e-10 * ((1.0 * (1000.0**1)) * (.001**-3) * (1.0**0))
if density != packet[0] or packet[1] != density:
    raise RuntimeError('Native HM_GET_FLOATV density factor order differs')
header = '// Executed native SDI direct/export receipt, law90-sdi-root-tests-2.\n#pragma once\nnamespace original_radiator_sdi {\n'
header += 'inline constexpr double density_kg_m3 = ' + density.hex() + ';\n'
header += 'inline constexpr double prepared[33] = {\n'
header += ''.join('  ' + float(value).hex() + ',\n' for value in packet) + '};\n}\n'
if (ROOT / 'source_fixture/NativeSdiOriginal.h').read_text() != header:
    raise RuntimeError('Executed SDI binary64 fixture differs')

spec = importlib.util.spec_from_file_location('native_preparation', ROOT / 'native/prepare_sources.py')
native = importlib.util.module_from_spec(spec)
spec.loader.exec_module(native)
with tempfile.TemporaryDirectory(prefix='law90-source-') as directory:
    native.prepare(Path(directory), False)
    native.prepare(Path(directory), True)
print('PASS: original28-point receipt, owned sources and complete native donor identities')
