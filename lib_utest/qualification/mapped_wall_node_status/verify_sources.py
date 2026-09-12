#!/usr/bin/env python3
"""Authenticate the complete old callers and unchanged floating-point work."""
from pathlib import Path
import hashlib
import json
import re
import subprocess
import sys

here = Path(__file__).resolve().parent
root = here.parents[2]
raw = (here / 'source-manifest.json').read_bytes()
assert hashlib.sha256(raw).hexdigest() == 'bf02626b1fcf6756367c090a00c25247aedfe565a28d7a722a4f80a5fd18a527'
manifest = json.loads(raw)
for row in manifest['files']:
    path = Path(row['path'])
    assert not path.is_absolute() and '..' not in path.parts
    data = (root / path).read_bytes()
    assert len(data) == row['bytes'] and hashlib.sha256(data).hexdigest() == row['sha256'], path

def original(name):
    text = (here / ('Frozen' + name + '.cuh')).read_text()
    replacements = {
        '"lib_src/collision/nodal_wall_mapped/Layout.h"': '"Layout.h"',
        '"lib_src/collision/NodalWallContactKernels.cuh"': '"../NodalWallContactKernels.cuh"',
        '"lib_src/collision/nodal_wall_mapped/ObserverReduction.cuh"': '"ObserverReduction.cuh"',
        '"lib_src/collision/nodal_wall_mapped/ScatterNode.h"': '"ScatterNode.h"',
        'nodal_wall_mapped::status_frozen': 'nodal_wall_mapped::parallel',
        'parallel::ReduceGlobalObservers(': 'ReduceGlobalObservers(',
    }
    for before, after in replacements.items():
        text = text.replace(before, after)
    record = manifest['original_callers'][name]
    data = text.encode()
    assert len(data) == record['bytes'] and hashlib.sha256(data).hexdigest() == record['sha256'], name
    return text

def body(text, name):
    # Skip the complete parameter list, which can include a default {} value.
    end = re.search(r'\b' + re.escape(name) + r'\(', text).end()
    depth = 1
    while depth:
        depth += (text[end] == '(') - (text[end] == ')')
        end += 1
    begin = text.index('{', end)
    end, depth = begin + 1, 1
    while depth:
        depth += (text[end] == '{') - (text[end] == '}')
        end += 1
    return text[begin + 1:end - 1]

def compact(text):
    return re.sub(r'\s+', '', re.sub(r'//[^\n]*', '', text))

wall = root / 'lib_src/collision/nodal_wall_mapped'
for name, functions in [('Evaluation', ('Points', 'Parents', 'Finish')),
                        ('Scatter', ('Stiffness', 'Forces', 'Publish'))]:
    old = original(name)
    current = (wall / (name + '.cuh')).read_text()
    for function in functions:
        actual = body(current, function).replace('SelectNodeFailure(storage, side, i);', '')
        assert compact(actual) == compact(body(old, function)), (name, function)
    assert 'atomicAdd' not in current and 'atomicMax' not in current

evaluation = (wall / 'Evaluation.cuh').read_text()
scatter = (wall / 'Scatter.cuh').read_text()
assert 'if (side.summary->points_admitted) side.summary->parent_failure = ~0ull;' in evaluation
assert 'if(!side.summary->points_admitted) return;' in body(evaluation, 'CheckPoints')
assert 'side.summary->parent_failure = ~0ull;' in body(evaluation, 'CheckPoints')
assert 'storage.control.status != NodalWallDeviceStatus::Ok' in body(scatter, 'CheckScatter')
assert 'CheckScatterNodes(storage);' in body(scatter, 'CheckScatter')
assert 'side.summary->parent_failure = ~0ull;' in body(scatter, 'CheckScatter')
select = (wall / 'NodeStatus.cuh').read_text()
assert 'atomicMin(&side.summary->parent_failure, static_cast<unsigned long long>(compact))' in select
assert 'atomicAdd' not in select and 'atomicMax' not in select
for text, function, stages in [
    (evaluation, 'Evaluate', ('BeginPoints', 'Points', 'CheckPoints', 'Parents')),
    (scatter, 'Scatter', ('BeginScatter', 'Stiffness', 'CheckScatter', 'Forces', 'CheckScatter', 'Publish')),
]:
    launch = body(text, function)
    offset = 0
    for stage in stages:
        offset = launch.index(stage + '<<<', offset) + len(stage) + 3
    for config in re.findall(r'<<<(.*?)>>>', launch):
        assert config.endswith(',stream'), config

# Complete layout/forecast/host operation hashes are retained above. Only the
# Summary lifetime comment changes; no allocation, view or coefficient changes.
subprocess.run([sys.executable, '-B', str(here.parent / 'mapped_wall_response/verify_sources.py')], check=True)
print(json.dumps({'status': 'passed', 'base': manifest['base'],
    'records': len(manifest['files']), 'added_host_bytes': 0, 'added_device_bytes': 0,
    'new_floating_arithmetic': False, 'numerical_execution': False}))
