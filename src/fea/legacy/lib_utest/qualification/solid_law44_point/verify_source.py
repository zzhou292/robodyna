#!/usr/bin/env python3
"""Frozen source/profile identities, generated curve and exact native excerpts."""
import hashlib
import importlib.util
import json
from pathlib import Path
import tempfile

ROOT = Path(__file__).resolve().parent
for item in json.loads((ROOT / 'source-manifest.json').read_text())['files']:
    value = (ROOT / item['path']).read_bytes()
    if hashlib.sha256(value).hexdigest() != item['sha256']:
        raise RuntimeError('Frozen source changed: ' + item['path'])
fixture = json.loads((ROOT / 'source_fixture/source-receipt.json').read_text())
xy = [row['text'].split() for row in fixture['curve']['cards'][1:]]
if len(xy) != 46 or {p['pid'] for p in fixture['parts']} != {2000016,2000392}:
    raise RuntimeError('Original source profile changed')
expected = ('#pragma once\nnamespace law44_solid_test {\ninline constexpr double X[]={'+
            ','.join(x for x,y in xy)+'};\ninline constexpr double Y[]={'+
            ','.join(y+'*1e6' for x,y in xy)+'};\ninline constexpr unsigned Count=46;\n}\n')
if (ROOT / 'source_fixture/OriginalCurve.h').read_text() != expected:
    raise RuntimeError('Original curve conversion changed')
spec = importlib.util.spec_from_file_location('prepare', ROOT / 'native/prepare_sources.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
with tempfile.TemporaryDirectory(prefix='solid-law44-sources-') as directory:
    path = Path(directory)
    module.prepare(path, False)
    module.prepare(path, True)
    if not (path / 'point_interface.inc').read_text().lstrip().startswith('SUBROUTINE L44S_REF_SIGEPS44('):
        raise RuntimeError('Native interface extraction boundary changed')
print('LAW44 source identities, original 46-point curve and complete native boundaries PASS')
