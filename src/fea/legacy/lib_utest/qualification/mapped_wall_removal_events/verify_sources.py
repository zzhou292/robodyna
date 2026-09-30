#!/usr/bin/env python3
"""Complete old caller, literal sum and reviewed scheduling/receipt chain."""
from pathlib import Path
import hashlib
import json
import re
import subprocess
import sys
from removal_proof import legacy_operations

here = Path(__file__).resolve().parent
root = here.parents[2]
raw = (here/'source-manifest.json').read_bytes()
assert hashlib.sha256(raw).hexdigest() == '17829697fb569bcc4d4824a7f1adcb480e01e62a229ebc499249041f65d8f81b'
manifest = json.loads(raw)
for row in manifest['files']:
    path = Path(row['path'])
    assert not path.is_absolute() and '..' not in path.parts
    data = (root/path).read_bytes()
    assert len(data) == row['bytes'] and hashlib.sha256(data).hexdigest() == row['sha256'], path

def function(text, start):
    begin = text.index(start)
    left = text.index('{', begin)
    end, depth = left + 1, 1
    while depth:
        depth += (text[end] == '{') - (text[end] == '}')
        end += 1
    return text[begin:end]

wall = root/'lib_src/collision/nodal_wall_mapped'
old = (here/'reference/Operations.cu').read_text()
ops = (wall/'Operations.cu').read_text()
assert legacy_operations(ops) == old
assert (here/'reference/Kernels.cuh').read_bytes() == (wall/'Kernels.cuh').read_bytes()
assert (here/'reference/Reduction.h').read_bytes() == (root/'lib_src/collision/NodalWallContactReduction.h').read_bytes()
frozen = (here/'Frozen.h').read_text()
removed = function((wall/'Kernels.cuh').read_text(), '__device__ inline bool RemovedPotential')
assert removed.replace('__device__', 'TL_SURFACE_HD') in frozen
finish = function(old, '__global__ void FinishCandidate')
assert finish.replace('__global__ void', 'TL_SURFACE_HD inline void').replace('m::RemovedPotential', 'RemovedPotential') in frozen
current = function(ops, '__global__ void FinishCandidate')
test = function((here/'CudaTest.cu').read_text(), '__global__ void Current')
assert re.sub(r'\s+', '', current).replace('FinishCandidate(', 'Current(') == re.sub(r'\s+', '', test)
values = (wall/'RemovalEvents.h').read_text()
assert 'accepted > 1 || proposed > accepted' in values
assert 'nodal_wall_reduction::Sum(' in values
assert 'while (events)' in values and 'events &= events - 1;' in values
assert 'sizeof(Tile) == 64' in values
kernel = (wall/'RemovalEvents.cuh').read_text()
assert 'Classify(accepted, side.proposed[parent])' in kernel
assert 'accepted > 1 ? Event::Invalid' in kernel
assert 'event == Event::Invalid' in kernel and 'event == Event::Removed' in kernel
assert kernel.count('__syncthreads()') == 2
assert 'storage.result.parents' not in kernel
assert 'atomic' not in kernel and 'atomic' not in values
subprocess.run([sys.executable, '-B', str(here.parent/'mapped_wall_node_status/verify_sources.py')], check=True)
print(json.dumps({'status': 'passed', 'records': len(manifest['files']),
    'base': manifest['base'], 'shared_bytes': 64, 'arena_bytes_added': 0,
    'numerical_execution': False}))
