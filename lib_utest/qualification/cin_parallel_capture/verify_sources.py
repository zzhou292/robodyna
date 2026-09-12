#!/usr/bin/env python3
"""Authenticate full old caller, exact copy extraction and unchanged owner budget."""
from pathlib import Path
import hashlib
import json
import re
import runpy
here = Path(__file__).resolve().parent
root = here.parents[2]
import runpy
group_proof = runpy.run_path(str(here.parent/"cin_parallel_groups/group_proof.py"))
raw = (here/'source-manifest.json').read_bytes()
assert hashlib.sha256(raw).hexdigest() == '79d7d2be3954bdac33bcfc8f816a04f73731531ad30c8caeac653a452550f3a6'
manifest = json.loads(raw)
for row in manifest['files']:
    path = Path(row['path'])
    assert not path.is_absolute() and '..' not in path.parts
    data = (root/path).read_bytes()
    assert len(data) == row['bytes'] and hashlib.sha256(data).hexdigest() == row['sha256'], path

proof = runpy.run_path(str(here/'capture_proof.py'))
old, loop = proof['REFERENCE'], proof['CAPTURE']
current = (root/'lib_src/solvers/ExplicitNodalCinStep.cu').read_text()
current = group_proof["legacy_owner"](current)
expected = proof['without_capture'](old)
expected = expected.replace('#include "cin_advance/Screen.h"', '#include "cin_advance/Screen.h"\n#include "cin_advance/Capture.h"')
old_launch = '  CompleteCin<<<1,1,0,stream>>>(input);\n  return cudaGetLastError();'
new_launch = '  CompleteCin<<<1,1,0,stream>>>(input);\n  error = cudaGetLastError();\n  if (error != cudaSuccess) return error;\n  return capture::Launch(input, stream);'
assert expected.count(old_launch) == 1
assert expected.replace(old_launch, new_launch) == current, 'only final capture extraction/launch may differ'
assert (here/'FrozenCaller.inc').read_text() == old[:old.index('cudaError_t cin_advance::Launch')].rstrip()+'\n'
launch = old[old.index('cudaError_t cin_advance::Launch'):old.index('\ncudaError_t FENodalState::Impl::LaunchCinAdvance')]
assert launch.replace('cudaError_t cin_advance::Launch', 'cudaError_t LaunchFrozen') in (here/'Frozen.cu').read_text()
assert loop in (here/'FrozenCopy.h').read_text(), 'complete old loop, independently compiled'

def body(value, name):
    start = value.index('{', value.index(name+'('))
    end, depth = start+1, 1
    while depth:
        depth += (value[end] == '{')-(value[end] == '}')
        end += 1
    return value[start+1:end-1]

def compact(value):
    return re.sub(r'\s+', '', value)

node = loop[loop.index('      if (groups.member_nodes'):loop.rfind('    }')]
node = node.replace('continue;', 'return;')
for name in ['groups', 'capture']:
    node = node.replace(name+'.', 'input.'+name+'.')
helper = (root/'lib_src/solvers/cin_advance/Capture.h').read_text()
copy = body(helper, 'CopyNode')
for line in ['  const auto n = input.model.node_count;\n',
             '  const auto* acceleration = input.work+3*n;\n',
             '  const auto* angular_acceleration = input.work+6*n;\n']:
    copy = copy.replace(line, '')
assert compact(copy) == compact(node), 'exact mask and six assignments'
kernels = (root/'lib_src/solvers/cin_advance/Capture.cu').read_text()
worker = body(kernels, 'Nodes')
assert worker.index('input.control->status != NodalStatus::Ok || !input.capture.node') < worker.index('for (std::uint32_t node')
assert 'node += blockDim.x*gridDim.x' in worker
assert 'CopyNode(input, node);' in worker
launch = body(kernels, 'Launch')
assert launch.index('if (!input.capture.node) return cudaSuccess;') < launch.index('Blocks(') < launch.index('Nodes<<<')
assert 'atomic' not in kernels and 'cudaMalloc' not in kernels
for file in ['CMakeLists.txt', 'BUILD.bazel']:
    assert 'cin_advance/Capture.cu' in (root/'lib_src/solvers'/file).read_text()
runpy.run_path(str(here.parent/'cin_parallel_screen/verify_sources.py'))
print(json.dumps({'status':'passed', 'records':len(manifest['files']),
    'new_device_bytes':0, 'new_host_bytes':0,
    'unchanged':['complete force/screen/ordinary/rigid/recovery/drift stages',
                 'member capture and partial errors', 'Input, layout, storage and owner budget'],
    'numerical_execution':False}))
