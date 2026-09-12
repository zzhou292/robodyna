#!/usr/bin/env python3
"""Authenticate unchanged native row expressions and complete frozen callers."""
from pathlib import Path
import hashlib
import json
import runpy

here = Path(__file__).resolve().parent
root = here.parents[2]
raw = (here/'source-manifest.json').read_bytes()
assert hashlib.sha256(raw).hexdigest() == '15c0710d6b0686e2fcc1bec132d1e1fdb991bcbc952d204caf73312d291d42f1'
manifest = json.loads(raw)
for row in manifest['files']:
    path = Path(row['path'])
    assert not path.is_absolute() and '..' not in path.parts
    data = (root/path).read_bytes()
    assert len(data) == row['bytes'] and hashlib.sha256(data).hexdigest() == row['sha256'], path
proof = runpy.run_path(str(here/'recovery_proof.py'))
witness = runpy.run_path(str(here.parent/'cin_limiter/witness_proof.py'))
proof['legacy_owner'](witness['legacy_owner']((root/'lib_src/solvers/ExplicitNodalCinStep.cu').read_text()))
old_motion = (here/'reference/CinMotionStage.h').read_text()
expected = old_motion.replace('#include "CinForceStage.h"', '#include "lib_src/constraints/tied_shell/runtime/CinForceStage.h"')
expected = expected.replace('#include "../TiedPatchMotion.h"', '#include "lib_src/constraints/tied_shell/TiedPatchMotion.h"')
expected = expected.replace('namespace tl::constraints::tied_shell::cin {', '''namespace tl::constraints::tied_shell::cin_recovery_frozen {
using cin::StageView;
using cin::MotionTrial;
using cin::StageReport;
using cin::StageStatus;
namespace detail { using cin::detail::ReadXyz; }''')
assert expected == (here/'FrozenMotion.h').read_text(), 'complete old recovery wrapper'
old_owner = (here/'reference/ExplicitNodalCinStep.cu').read_text()
expected = old_owner[:old_owner.index('\ncudaError_t FENodalState::Impl::LaunchCinAdvance')]
expected += '\n} // namespace tl::fea::cin_recovery_test\n'
expected = expected.replace('namespace tl::fea {', 'namespace tl::fea::cin_recovery_test {\nusing namespace cin_advance;')
expected = expected.replace('#include "../constraints/tied_shell/runtime/CinMotionStage.h"', '#include "FrozenMotion.h"')
expected = expected.replace('cin::RecoverMotionTrial(', 'constraints::tied_shell::cin_recovery_frozen::RecoverMotionTrial(')
expected = expected.replace('cudaError_t cin_advance::Launch(', 'cudaError_t LaunchFrozen(')
assert expected == (here/'Frozen.cu').read_text(), 'complete frozen CUDA caller'
body, same = proof['body'], proof['same']
same(body((here/'FrozenTail.cuh').read_text(), 'CompleteCin'),
     body(expected, 'CompleteCin').replace('cin::StageReport stage;', 'constraints::tied_shell::cin::StageReport stage;'),
     'complete old tail used by post-kick failure tests')
kernels = (root/'lib_src/solvers/cin_advance/Recovery.cu').read_text()
for name in ('Begin', 'PrepareRows', 'PublishRows', 'Complete'):
    assert body(kernels, name).lstrip().startswith('if (input.control->status != NodalStatus::Ok) return;')
assert body(kernels, 'Begin').index('MotionPointersValid') < body(kernels, 'Begin').index('*input.recovery_failure = NoFailure;')
assert 'atomicMin(input.recovery_failure, row)' in body(kernels, 'PrepareRows')
assert 'input.prepared_recovery[row] = result;' in body(kernels, 'PrepareRows')
assert 'cudaMalloc' not in kernels
launch = body(kernels, 'Launch')
assert launch.index('Begin<<<') < launch.index('PrepareRows<<<') < launch.index('PublishRows<<<') < launch.index('Complete<<<')
helper = (root/'lib_src/solvers/cin_advance/Recovery.h').read_text()
same(body(helper, 'Prepare'), '''Row result;
  result.valid = bool(cin::detail::PrepareMotionRow(input.model, MotionView(input), row, result.motion));
  return result;''', 'fresh exact leaves, no destination writes')
same(body(helper, 'Publish'), '''if (row < first_failure)
  cin::detail::ApplyMotionRow(input.model, MotionView(input), row, input.prepared_recovery[row].motion);''',
     'publish exactly the old successful source-row prefix')
finish = body(kernels, 'Complete')
assert finish.index('failed != NoFailure') < finish.index('Drift(input)')
owner = (root/'lib_src/solvers/ExplicitNodalCinStep.cu').read_text()
launch = body(owner, 'cin_advance::Launch')
assert launch.index('groups::LaunchMotion') < launch.index('CompleteCin<<<') < launch.index('recovery::Launch') < launch.index('capture::Launch')
assert 'cin->prepared_recovery, cin->recovery_failure, cin->prepared_drift}, stream)' in owner
startup = (root/'lib_src/solvers/NodalCinStartup.cpp').read_text()
assert 'if (next->dependent[row.secondary])' in startup
assert 'if (next->dependent[node])' in startup
assert 'rigid/CIN' in startup or 'CIN and rigid' in startup
runpy.run_path(str(here.parent/'cin_force_transfers/verify_sources.py'))
print(json.dumps({'status':'passed','records':len(manifest['files']),
    'baseline':manifest['baseline_commit'],'numerical_execution':False,
    'new_device_bytes_per_row':104,'new_device_key_bytes':4,'per_step_allocations':0,
    'unchanged':['native recovery expressions and repeated slots','first source-row failure and four-field prefix',
                 'all force/ordinary/rigid stages','complete drift/orientation order','capture and accepted transaction']}))
