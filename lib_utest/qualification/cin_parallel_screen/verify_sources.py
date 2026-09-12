#!/usr/bin/env python3
"""Authenticate the complete old Screen/caller and exact helper extraction."""
from pathlib import Path
import hashlib
import json
import re
import runpy

here = Path(__file__).resolve().parent
root = here.parents[2]
import runpy
group_proof = runpy.run_path(str(here.parent/"cin_parallel_groups/group_proof.py"))
raw = (here / 'source-manifest.json').read_bytes()
assert hashlib.sha256(raw).hexdigest() == '8174a46fa85a429405478f28b5462363a4c700560c463f5f1053fefdb0adc177'
manifest = json.loads(raw)
for row in manifest['files']:
    path = Path(row['path'])
    assert not path.is_absolute() and '..' not in path.parts
    data = (root / path).read_bytes()
    assert len(data) == row['bytes'] and hashlib.sha256(data).hexdigest() == row['sha256'], path

def body(text, name):
    start = text.index('{', text.index(name+'('))
    end, depth = start+1, 1
    while depth:
        depth += (text[end] == '{')-(text[end] == '}')
        end += 1
    return text[start+1:end-1]

def compact(text):
    return re.sub(r'\s+', '', text)

def same(a, b, label):
    assert compact(a) == compact(b), label

old = (here/'reference/Screen.h').read_text()
frozen = old.replace('#include "Rigid.h"', '#include "lib_src/solvers/cin_timestep/Rigid.h"')
frozen = frozen.replace('namespace tl::fea::cin_timestep {', 'namespace tl::fea::cin_screen_frozen {\nusing cin_timestep::ScalarLimit;\nusing cin_timestep::OrdinaryLimit;\nusing cin_timestep::AddRigidMemberTrace;\nusing cin_timestep::RigidTraceLimit;')
frozen = frozen.replace('} // namespace tl::fea::cin_timestep', '} // namespace tl::fea::cin_screen_frozen')
assert frozen == (here/'FrozenScreen.h').read_text(), 'complete frozen Screen'
sources = (root/'lib_src/solvers/cin_timestep/Sources.h').read_text()
for name in ('Sources', 'Result'):
    start = 'struct '+name+' {'
    a, b = old.index(start), sources.index(start)
    same(old[a:old.index('};', a)+2], sources[b:sources.index('};', b)+2], name)
same(body(old, 'Include'), body(sources, 'Include'), 'strict Include sentinel/ties')
values = group_proof["legacy_values"]((root/'lib_src/solvers/cin_timestep/ScreenValues.h').read_text())
check = body(old, 'Screen')
node_start = check.index('  for (std::uint32_t node')
group_start = check.index('  for (std::uint32_t group')
finish = check.index('  next.valid = true;')
header = check[:check.index('  Result next;')]
same(body(values, 'CheckSources'), header+'return true;', 'complete header/read order')
node = check[node_start:group_start]
node = node[node.index('{')+1:node.rfind('}')]
node = node.replace('    invalid_node = node;', '').replace('continue;', 'return true;')
same(body(values, 'EvaluateNode'), 'const auto groups = source.rigid;'+node+'return true;', 'complete ordinary operation/readset')
same(body(values, 'EvaluateGroups'), 'const auto groups = source.rigid;'+check[group_start:finish]+'return true;', 'complete ordered rigid suffix')
serial = (root/'lib_src/solvers/cin_timestep/Screen.h').read_text()
same(body(serial, 'Screen'), '\n  if (!detail::CheckSources(source, factor)) return false;\n  Result next;\n  for (std::uint32_t node = 0; node < source.nodes; ++node) {\n    invalid_node = node;\n    if (!detail::EvaluateNode(source, factor, node, next)) return false;\n  }\n  if (!detail::EvaluateGroups(source, factor, next, invalid_node)) return false;\n  next.valid = true;\n  invalid_node = UINT32_MAX;\n  output = next;\n  return true;\n', 'legacy serial wrapper')
old_owner = (here/'reference/ExplicitNodalCinStep.cu').read_text()
extract = old_owner[:old_owner.index('cudaError_t cin_advance::Launch')]
extract = extract.replace('#include "cin_timestep/Screen.h"', '#include "FrozenScreen.h"')
extract = extract.replace('cin_timestep::', 'cin_screen_frozen::')
assert extract == (here/'FrozenCaller.inc').read_text(), 'complete frozen caller prefix/motion/suffix'
same(body(old_owner, 'cin_advance::Launch'), body((here/'Frozen.cu').read_text(), 'LaunchFrozen'), 'frozen launch order')
owner = (root/'lib_src/solvers/ExplicitNodalCinStep.cu').read_text()
owner = group_proof["legacy_owner"](owner)
same(body(owner, 'AdvanceOrdinaryCin'), body(old_owner, 'AdvanceOrdinaryCin'), 'ordinary motion')
without_capture = runpy.run_path(str(here.parent/'cin_parallel_capture/capture_proof.py'))['without_capture']
same(body(owner, 'CompleteCin'), without_capture(body(old_owner, 'CompleteCin')), 'ordered suffix before exact final copy extraction')
current_prefix = body(owner, 'PrepareCin')
insert = '  // Screen owns the ordinary failure-key initialization only after successful\n  // structural admission. A rejected screen leaves the prior key untouched.\n  if (parallel_screen) return;\n'
assert insert in current_prefix
same(current_prefix.replace(insert, ''), body(old_owner, 'PrepareCin'), 'unchanged force/serial preparation')
launch = body(owner, 'cin_advance::Launch')
assert launch.index('force_inputs::Launch') < launch.index('PrepareCin<<<') < launch.index('screen::Launch') < launch.index('AdvanceOrdinaryCin<<<') < launch.index('CompleteCin<<<')
assert 'input.screen && screen::Blocks(input.model.node_count) &&' in launch
assert 'cin->failure, cin->input_failure, cin->screen}, stream)' in owner
storage = (root/'lib_src/solvers/NodalCinStorage.cu').read_text()
assert 'screen = util::ArenaPointer<cin_advance::screen::Summary>(arena, layout.screen);' in storage
layout = (root/'lib_src/solvers/NodalCinLayout.h').read_text()
assert 'device.Append<cin_advance::screen::Summary>(cin_advance::screen::Blocks(n), next.screen)' in layout
summary = (root/'lib_src/solvers/cin_advance/ScreenSummary.h').read_text()
assert 'sizeof(Summary) == 16' in summary and 'MaximumBlocks = 256' in summary
assert 'invalid_node = source.nodes-1;' in summary
assert summary.index('summary.invalid_node != UINT32_MAX') < summary.index('EvaluateGroups(')
kernels = group_proof["legacy_kernels"]((root/'lib_src/solvers/cin_advance/Screen.cu').read_text())
for name in ('Begin', 'Nodes', 'Finish'):
    assert body(kernels, name).lstrip().startswith('if (input.control->status != NodalStatus::Ok) return;')
finish = body(kernels, 'Finish')
assert finish.index('Complete(') < finish.index('input.control->limit.dt =') < finish.index('input.durations.drift_dt >') < finish.index('*input.failure = NoFailure;')
assert 'atomic' not in kernels
assert 'input.screen[blockIdx.x] = values[0];' in kernels
runpy.run_path(str(here.parent/'cin_parallel_ordinary/verify_sources.py'))
print(json.dumps({'status':'passed', 'records':len(manifest['files']),
    'unchanged':['ordinary scalar arithmetic', 'full node consumed readset', 'group/member trace order',
                 'force prefix', 'ordinary motion', 'rigid/recovery/drift/capture suffix'],
    'new_device_bytes_max':4096, 'numerical_execution':False}))
