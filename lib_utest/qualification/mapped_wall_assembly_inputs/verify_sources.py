#!/usr/bin/env python3
"""Preserve complete mapped validator, caller order and all later arithmetic."""
from pathlib import Path
import hashlib, json, re, subprocess, sys
here = Path(__file__).resolve().parent
root = here.parents[2]
raw = (here/'source-manifest.json').read_bytes()
assert hashlib.sha256(raw).hexdigest() == '39adaf1756e57cc6c3094480cb4e2210f413dafd01c29e1ed056aeea9fc0603c'
manifest = json.loads(raw)
for row in manifest['files']:
    path = Path(row['path'])
    assert not path.is_absolute() and '..' not in path.parts
    data = (root/path).read_bytes()
    assert len(data) == row['bytes'] and hashlib.sha256(data).hexdigest() == row['sha256'], path

def body(text, name):
    start = text.index('{', text.index(name+'('))
    end, depth = start+1, 1
    while depth:
        depth += (text[end] == '{')-(text[end] == '}')
        end += 1
    return text[start+1:end-1]

def compact(text):
    text = re.sub(r'//[^\n]*', '', text)
    return re.sub(r'\s+', '', text)

def same(a, b, name):
    assert compact(a) == compact(b), name

folder = root/'lib_src/collision/nodal_wall_mapped'
old = (here/'reference/Kernels.cuh').read_text()
current = (folder/'Kernels.cuh').read_text()
frozen = (here/'FrozenValidation.h').read_text()
same(body(frozen, 'ValidateAssembly'), body(old, 'ValidateAssembly'), 'complete frozen validator')
value = body(old, 'ValidateAssembly')
node_start = value.index('  for(unsigned i=0;')
helpers = (folder/'AssemblyValidation.h').read_text()
header = value[:node_start].replace('storage.control', 'status')
same(body(helpers, 'Header'), header+'return true;', 'complete header read/priority')
mass = value[value.index('    const auto node='):value.index('    side.inverse[i]=inverse;')]
mass = mass.replace('[i]', '[compact]').replace('const auto inverse=', 'inverse=')
mass = mass.replace('storage.control', 'status')
same(body(helpers, 'Mass'), mass+'return true;', 'complete inverse/mask/root priority')
geometry = value[value.index('    if(view.accepted.base_epoch==0)'):value.rfind('  }')]
geometry = geometry.replace('if(view.accepted.base_epoch==0) {', 'if(view.accepted.base_epoch==0) {const auto node=storage.model.nodes[compact].node;')
geometry = geometry.replace('storage.control', 'status')
same(body(helpers, 'Geometry'), geometry+'return true;', 'epoch-zero consumed coordinate readset')
same(body(current, 'ValidateAssembly'), '\n  namespace a = assembly_validation;\n  if (!a::Header(view, storage.control)) return false;\n  for (unsigned i = 0; i < storage.model.node_count; ++i) {\n    double inverse = 0;\n    if (!a::Mass(storage, side, view, i, inverse, storage.control)) return false;\n    side.inverse[i] = inverse;\n    if (!a::Geometry(storage, view, i, storage.control)) return false;\n  }\n  return true;\n', 'legacy serial wrapper and exact inverse publication point')
for name in ('Response', 'StageStiffness', 'RemovedPotential'):
    same(body(old, name), body(current, name), name)
old_ops = (here/'reference/Operations.cu').read_text()
ops = (folder/'Operations.cu').read_text()
sys.path.insert(0, str(here.parent/'mapped_wall_removal_events'))
from removal_proof import legacy_operations
ops = legacy_operations(ops)
for name in ('CheckResponse', 'FinishAssembly', 'CopyAcceptedBase', 'BeginCandidate',
             'FinishCandidate', 'Identity', 'Check', 'ReadControl', 'EvaluateCandidate'):
    same(body(old_ops, name), body(ops, name), name)
accepted = body(old_ops, 'AssembleAccepted')
accepted = accepted.replace('BeginAssembly<<<1,1,0,state.stream>>>(state.device,state.remote,view);',
    'm::assembly_validation::Launch(state.device,state.remote,view,nodes,state.stream);')
accepted = accepted.replace('CheckResponse<<<1,1,0,state.stream>>>(state.device,state.remote,view);',
    'm::response::Launch(state.device,state.remote,view,state.stream);')
same(accepted, body(ops, 'AssembleAccepted'), 'complete caller and all unchanged post-validation stage order')
layout = (folder/'Layout.h').read_text()
for addition in ('#include "ResponseTypes.h"\n', '  response::Scratch response;\n',
    '  tl::util::ArenaRegion response_offsets,response_rows,response_maxima;\n'):
    assert addition in layout
    layout = layout.replace(addition, '')
same((here/'reference/Layout.h').read_text(), layout, 'old layout prefix; separate response tail owns its growth')
kernels = (folder/'AssemblyValidation.cuh').read_text()
launch = body(kernels, 'Launch')
assert launch.index('Begin<<<') < launch.index('Nodes<<<') < launch.index('CopyInversePrefix<<<') < launch.index('Finish<<<')
assert 'atomicMin(&side.summary->parent_failure, Failure(i, false))' in kernels
assert 'else if (!Geometry(' in kernels
assert 'atomicMin(&side.summary->parent_failure, Failure(i, true))' in kernels
assert 'if (!side.summary->points_admitted) return;' in body(kernels, 'Nodes')
assert 'if (!side.summary->points_admitted) return;' in body(kernels, 'CopyInversePrefix')
assert 'side.inverse[i] = view.mass.inverse_mass[node];' in kernels
assert 'side.summary->parent_failure = NoFailure;' in body(helpers, 'Complete')
assert 'failure/2+(failure&1)' in body(helpers, 'InversePrefix')
subprocess.run([sys.executable, '-B', str(here.parent/'mapped_wall_interval/verify_sources.py')], check=True)
subprocess.run(['cmake', '-DTL_ROOT='+str(root), '-P', str(here.parent/'mapped_wall_scatter/Verify.cmake')], check=True)
print(json.dumps({'status':'passed','records':len(manifest['files']),
    'assembly_input_new_arena_bytes':0,'assembly_input_new_host_metadata_bytes':0,
    'unchanged':['header/mass/geometry read order','partial inverse prefix','caller suffix except separately qualified response launch',
                 'response','scatter','activity','candidate/interval arithmetic'],
    'numerical_execution':False}))
