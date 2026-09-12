#!/usr/bin/env python3
"""Authenticate frozen dependent drift, orientation and exact phase ordering."""
from pathlib import Path
import hashlib
import json
import runpy

here = Path(__file__).resolve().parent
root = here.parents[2]
raw = (here/'source-manifest.json').read_bytes()
assert hashlib.sha256(raw).hexdigest() == '51f145016b37ed7ce86548c2c51f1c2ef25095b635b535e965a0c3d83d913a0f'
manifest = json.loads(raw)
for row in manifest['files']:
    path = Path(row['path'])
    assert not path.is_absolute() and '..' not in path.parts
    data = (root/path).read_bytes()
    assert len(data) == row['bytes'] and hashlib.sha256(data).hexdigest() == row['sha256'], path
proof = runpy.run_path(str(here/'drift_proof.py'))
restored = proof['legacy_owner']((root/'lib_src/solvers/ExplicitNodalCinStep.cu').read_text())
assert restored == (here/'reference/ExplicitNodalCinStep.cu').read_text()
body, same = proof['body'], proof['same']
old = (here/'reference/NodalNodeStep.h').read_text()
expected = old.replace('#include "FENodalStateStorage.h"', '#include "lib_src/solvers/FENodalStateStorage.h"')
expected = expected.replace('#include "NodalRotation.h"', '#include "lib_src/solvers/NodalRotation.h"')
expected = expected.replace('namespace tl::fea::nodal_detail {',
    'namespace tl::fea::cin_drift_test::frozen_orientation {\nusing namespace tl::fea::nodal_detail;')
assert expected == (here/'FrozenOrientation.h').read_text(), 'complete original orientation, not a mirrored new implementation'
old = (here/'reference/RecoveryDrift.h').read_text()
expected = old.replace('#include "Input.h"', '#include "lib_src/solvers/cin_advance/Input.h"')
expected = expected.replace('#include "../NodalNodeStep.h"', '#include "FrozenOrientation.h"')
expected = expected.replace('namespace tl::fea::cin_advance::recovery {',
    'namespace tl::fea::cin_drift_test::frozen {\nusing cin_advance::Input;')
expected = expected.replace('nodal_detail::PrepareNodeOrientation(', 'frozen_orientation::PrepareNodeOrientation(')
assert expected == (here/'FrozenDrift.h').read_text(), 'complete old dependent drift, including failing-axis stores'
old = (here/'reference/ExplicitNodalCinStep.cu').read_text()
expected = old[:old.index('\ncudaError_t FENodalState::Impl::LaunchCinAdvance')]
expected += '\n} // namespace tl::fea::cin_drift_test\n'
expected = '#include "FrozenDrift.h"\n'+expected
expected = expected.replace('namespace tl::fea {', 'namespace tl::fea::cin_drift_test {\nusing namespace cin_advance;')
expected = expected.replace('cudaError_t cin_advance::Launch(', 'cudaError_t LaunchFrozen(')
expected = expected.replace('cin_advance::recovery::Drift(input);', 'frozen::Drift(input);')
expected = expected.replace('nodal_detail::PrepareNodeOrientation(', 'frozen_orientation::PrepareNodeOrientation(')
expected = expected.replace('nodal_detail::AdvanceOrdinaryNode<', 'frozen_orientation::AdvanceOrdinaryNode<')
assert expected == (here/'Frozen.cu').read_text(), 'complete frozen caller with old orientation/drift'
expected = (here.parent/'cin_parallel_recovery/FrozenTail.cuh').read_text()
expected = expected.replace('cin_recovery_test::frozen_tail', 'cin_drift_test::frozen_tail')
expected = expected.replace('nodal_detail::PrepareNodeOrientation(', 'frozen_orientation::PrepareNodeOrientation(')
assert expected == (here/'FrozenTail.cuh').read_text(), 'complete earlier recovery/drift tail with old orientation'
values = (root/'lib_src/solvers/cin_advance/DriftValues.h').read_text()
same(body(values, 'Prepare'), '''
  Row result;
  const auto n = input.model.node_count;
  const auto node = input.model.rows[row].secondary;
  for (unsigned axis = 0; axis < 3; ++axis) {
    const auto index = 3*node+axis;
    result.position[axis] = input.accepted[index]+input.durations.drift_dt*input.trial[3*n+index];
    result.stored_axes = axis+1;
    if (!std::isfinite(result.position[axis])) {
      result.status = NodalStatus::InvalidOutput;
      return result;
    }
  }
  result.status = nodal_detail::PrepareNodeOrientationValue(input.accepted,
      input.trial, node, n, input.durations.drift_dt, input.maximum_angle,
      false, result.orientation);
  return result;
''', 'same-node leaves, unchanged expression/check order, no reaction/position dependencies')
same(body(values, 'Publish'), '''
  if (row > first_failure) return;
  const auto& result = input.prepared_drift[row];
  const auto n = input.model.node_count;
  const auto node = input.model.rows[row].secondary;
  for (unsigned axis = 0; axis < result.stored_axes; ++axis) {
    const auto index = 3*node+axis;
    input.trial[index] = result.position[axis];
    input.trial[13*n+index] = 0;
    input.trial[16*n+index] = 0;
  }
  if (result.status != NodalStatus::Ok) return;
  auto* quaternion = input.trial+9*n+4*node;
  quaternion[0] = result.orientation.w;
  quaternion[1] = result.orientation.x;
  quaternion[2] = result.orientation.y;
  quaternion[3] = result.orientation.z;
''', 'source-row prefix includes failing axis/reactions and only successful quaternions')
kernels = (root/'lib_src/solvers/cin_advance/Drift.cu').read_text()
for name in ('Begin', 'PrepareRows', 'PublishRows'):
    assert body(kernels, name).lstrip().startswith('if (input.control->status != NodalStatus::Ok) return;')
assert '*input.recovery_failure = recovery::NoFailure;' in body(kernels, 'Begin')
assert 'atomicMin(input.recovery_failure, row)' in body(kernels, 'PrepareRows')
assert 'input.prepared_drift[row] = result;' in body(kernels, 'PrepareRows')
assert 'cudaMalloc' not in kernels
launch = body(kernels, 'Launch')
assert launch.index('Begin<<<') < launch.index('PrepareRows<<<') < launch.index('PublishRows<<<') < launch.index('CompleteRows<<<')
assert body(values, 'Complete').lstrip().startswith('if (input.control->status != NodalStatus::Ok) return;')
startup = (root/'lib_src/solvers/NodalCinStartup.cpp').read_text()
assert 'if (next->dependent[row.secondary])' in startup
assert 'if (next->dependent[node])' in startup
storage = (root/'lib_src/solvers/NodalCinStorage.cu').read_text()
assert 'prepared_drift = util::ArenaPointer<cin_advance::drift::Row>(arena, layout.prepared_drift);' in storage
assert 'cin->recovery_failure, cin->prepared_drift}, stream)' in (root/'lib_src/solvers/ExplicitNodalCinStep.cu').read_text()
runpy.run_path(str(here.parent/'cin_parallel_recovery/verify_sources.py'))
print(json.dumps({'status':'passed', 'records':len(manifest['files']),
    'baseline':manifest['baseline_commit'], 'numerical_execution':False,
    'device_payload_bytes_per_row':64, 'additional_key_bytes':0, 'per_step_allocations':0,
    'unchanged':['ordinary and rigid arithmetic', 'recovery error precedence',
      'first source-row failure and failing-axis stores', 'native quaternion and limit',
      'capture, limiter and accepted transaction']}))
