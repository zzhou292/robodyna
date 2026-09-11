#!/usr/bin/env python3
"""Authenticate complete old force caller and exact extraction/phase order."""
from pathlib import Path
import hashlib
import json
import re

here = Path(__file__).resolve().parent
root = here.parents[2]
raw = (here / 'source-manifest.json').read_bytes()
assert hashlib.sha256(raw).hexdigest() == '5d8e6e05376b0b1275c38171c3ff5a9a139fb46f59e6d8eeb6502dba3b2603b3'
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

old = (here/'serial/CinForceStage.h.txt').read_text()
current = (root/'lib_src/constraints/tied_shell/runtime/CinForceStage.h').read_text()
frozen = old.replace('#include "CinStageTypes.h"', '#include "lib_src/constraints/tied_shell/runtime/CinStageTypes.h"')
frozen = frozen.replace('#include "../TiedPatchForce.h"', '#include "lib_src/constraints/tied_shell/TiedPatchForce.h"')
frozen = frozen.replace('namespace tl::constraints::tied_shell::cin {', 'namespace tl::constraints::tied_shell::cin_input_frozen {\nusing namespace cin;')
frozen = frozen.replace('} // namespace tl::constraints::tied_shell::cin', '} // namespace tl::constraints::tied_shell::cin_input_frozen')
assert frozen == (here/'serial/Force.h').read_text(), 'complete frozen force header'
for name in ('ReadXyz', 'Nonnegative'):
    same(body(current, name), body(old, name), name)
checks = body(old, 'CheckForceInputs')
node_start = checks.index('  for (std::uint32_t i')
node_end = checks.index('  if (!tied_shell::detail::math::Finite(*trial.numerical_mass))')
same(body(current, 'CheckForcePointers'), checks[:node_start]+'  return {};', 'pointer order')
loop = checks[node_start:node_end]
loop = loop[loop.index('{')+1:loop.rfind('}')]
same(body(current, 'CheckForceNode'), loop+'  return {};', 'complete node order')
same(body(current, 'CheckForceAfterNodes'), checks[node_end:], 'numerical/witness/row order')
same(body(current, 'CheckForceInputs'), '''
  auto report = CheckForcePointers(model, trial);
  if (!report) return report;
  for (std::uint32_t node = 0; node < model.node_count; ++node) {
    report = CheckForceNode(model, trial, node);
    if (!report) return report;
  }
  return CheckForceAfterNodes(model, trial);
''', 'serial check wrapper')
force = body(old, 'PrepareForceTrial')
transfer_start = force.index('  for (std::uint32_t r')
same(body(current, 'TransferForceTrial'), '  const auto n = model.node_count;\n'+force[transfer_start:], 'ordered transfer')
same(body(current, 'PrepareForceTrial'), force[:transfer_start]+'  return detail::TransferForceTrial(model, trial);', 'serial copy/transfer wrapper')

old_owner = (here/'serial/ExplicitNodalCinStep.cu.txt').read_text()
owner = (root/'lib_src/solvers/ExplicitNodalCinStep.cu').read_text()
inputs = (root/'lib_src/solvers/cin_advance/ForceInputs.h').read_text()
prefix_start = old_owner.index('  if (control->status != NodalStatus::Ok) return;')
prefix_end = old_owner.index('  const auto n = model.node_count;', prefix_start)
prefix = old_owner[prefix_start:prefix_end]
prefix = prefix.replace('return;', 'return false;')
prefix = prefix.replace('Fail(control, NodalStatus::StaleTrial, UINT32_MAX);', 'control->status = NodalStatus::StaleTrial; control->node = UINT32_MAX;')
prefix = prefix.replace('Fail(control, NodalStatus::MissingStepAdmission, UINT32_MAX);', 'control->status = NodalStatus::MissingStepAdmission; control->node = UINT32_MAX;')
for name in ('epoch', 'attempt', 'durations'):
    prefix = re.sub(r'(?<![.\w])'+name+r'\b', 'input.'+name, prefix)
same(body(inputs, 'CheckPrefix'), 'auto* control = input.control;'+prefix+'return true;', 'owner prefix')
start, end = '  if (structural.profile != NodalCinStructuralProfile::Disabled)', '  *input.failure = cin_advance::NoFailure;'
assert old_owner[old_owner.index(start):old_owner.index(end)] == owner[owner.index(start):owner.index(end)], 'structural screen'
for name in ('AdvanceOrdinaryCin', 'CompleteCin'):
    same(body(owner, name), body(old_owner, name), name)
kernels = (root/'lib_src/solvers/cin_advance/ForceInputs.cu').read_text()
launch = body(kernels, 'Launch')
assert launch.index('BeginInputs<<<') < launch.index('CheckNodes<<<') < launch.index('CompleteInputs<<<') < launch.index('CopyEntryInertia<<<')
for name in ('CheckNodes', 'CopyEntryInertia'):
    value = body(kernels, name)
    assert value.index('input.control->status != NodalStatus::Ok') < value.index('const auto force')
    assert 'node += gridDim.x*blockDim.x' in value
assert 'atomicMin(input.input_failure, static_cast<FailureKey>(node))' in kernels
assert 'force.entry_inertia[node] = force.inertia[node];' in kernels
assert 'force_inputs::Launch(input, stream)' in owner
assert 'cin->failure, cin->input_failure}, stream)' in owner
for file in ('Serial.cu', 'SerialHost.cpp'):
    value = (here/file).read_text()
    assert '#define PrepareForceTrial PrepareFrozenForceTrial' in value
    assert '#include "../cin_parallel_ordinary/serial/Kernel.inc"' in value
assert 'return cin_input_frozen::PrepareForceTrial(model, trial);' in (here/'Serial.h').read_text()
print(json.dumps({'status':'passed','records':len(manifest['files']),
    'unchanged':['force input checks','serial wrapper','ordered transfer','Screen','ordinary motion','rigid/recovery/capture'],
    'numerical_execution':False}))
