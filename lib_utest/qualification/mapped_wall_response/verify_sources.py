#!/usr/bin/env python3
"""Complete frozen response, admitted CSR order and unchanged caller suffix."""
from pathlib import Path
import hashlib, json, re, subprocess, sys
here = Path(__file__).resolve().parent
root = here.parents[2]
raw = (here/'source-manifest.json').read_bytes()
assert hashlib.sha256(raw).hexdigest() == 'f4bb6eb52342949a3055926852e7e153244a8c2e01d40d63152895262931fbdb'
for row in json.loads(raw)['files']:
    path = Path(row['path'])
    assert not path.is_absolute() and '..' not in path.parts
    value = (root/path).read_bytes()
    assert len(value) == row['bytes'] and hashlib.sha256(value).hexdigest() == row['sha256'], path

def body(text, name):
    begin = text.index('{', text.index(name+'(')); end = begin+1; depth = 1
    while depth:
        depth += (text[end] == '{')-(text[end] == '}'); end += 1
    return text[begin+1:end-1]

def compact(text):
    return re.sub(r'\s+', '', re.sub(r'//[^\n]*', '', text))

def same(a, b, label):
    assert compact(a) == compact(b), label

wall = root/'lib_src/collision/nodal_wall_mapped'
old = (here/'reference/Kernels.cuh').read_text()
current = (wall/'Kernels.cuh').read_text()
frozen = (here/'FrozenResponse.h').read_text()
same(old, current, 'complete old kernels and serial fallback unchanged')
same(body(old, 'Response'), body(frozen, 'Response'), 'complete frozen response')
old_ops = (here/'reference/Operations.cu').read_text()
ops = (wall/'Operations.cu').read_text()
same(body(old_ops, 'CheckResponse').replace('m::Response', 'Response'),
    body(frozen, 'CheckResponse'), 'complete frozen check')
expected = old_ops.replace('#include "Kernels.cuh"', '#include "Kernels.cuh"\n#include "Response.cuh"')
expected = expected.replace('CheckResponse<<<1,1,0,state.stream>>>(state.device,state.remote,view);',
    'm::response::Launch(state.device,state.remote,view,state.stream);')
same(expected, ops, 'one launch replacement; every other operation unchanged')
values = (wall/'ResponseValues.h').read_text()
step = body(old_ops, 'CheckResponse')
step = step[step.index('    double step='):step.rfind('  }')]
same(step, body(values, 'CheckStep'), 'exact step arithmetic and threshold')
response = (wall/'Response.cuh').read_text()
launch = body(response, 'Launch')
assert launch.index('Begin<<<') < launch.index('OrdinaryNodes<<<') < launch.index('RigidGroups<<<') < launch.index('Finish<<<')
assert 'CompleteRate(storage, side) || nodal_wall_mapped::Response(storage, side, k)' in response
assert 'atomicAdd' not in response and 'atomicMax' not in response
assert response.count('atomicMin') == 2
for name in ('OrdinaryNodes', 'RigidGroups', 'Finish'):
    assert 'if (storage.control.status != Code::Ok) return;' in body(response, name)
assert 'if (upper == 0) return true;' in body(values, 'Ordinary')
assert 'if (upper == 0) continue;' in body(values, 'Rigid')
assert 'AccumulateRigidContactTrace(upper, response, side.traces[group])' in values
assert 'for (auto slot = begin; slot < end; ++slot)' in values
assert 'side.summary->parent_failure = NoFailure;' in body(values, 'CompleteRate')
assert 'sizeof(Impl)' in (wall/'Forecast.cpp').read_text()
for name in ('response_offsets', 'response_rows', 'response_maxima'):
    assert 'layout.'+name in (wall/'Initialize.cpp').read_text()
assert 'BuildIncidence(local.roots,weights.node_count(),local.groups,local.response)' in (wall/'Sources.cpp').read_text()
subprocess.run([sys.executable, '-B', str(here.parent/'mapped_wall_assembly_inputs/verify_sources.py')], check=True)
print(json.dumps({'status':'passed', 'frozen_revision':'3e0d1d4',
    'unchanged':['serial error priority/partial values', 'rigid trace row order', 'step formula',
                 'force/stiffness scatter', 'activity', 'candidate/interval', 'legacy API'],
    'numerical_execution':False}))
