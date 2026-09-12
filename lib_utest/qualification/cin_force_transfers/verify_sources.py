#!/usr/bin/env python3
"""Authenticate full old callers and the exact pure-leaf/ordered-apply split."""
from pathlib import Path
import hashlib
import json
import runpy

here = Path(__file__).resolve().parent
root = here.parents[2]
raw = (here/'source-manifest.json').read_bytes()
assert hashlib.sha256(raw).hexdigest() == 'bd70bb5ee0d0af28062764731b86b0680b0c83255f52dd40b47ba6b58655ec8d'
manifest = json.loads(raw)
for row in manifest['files']:
    path = Path(row['path'])
    assert not path.is_absolute() and '..' not in path.parts
    data = (root/path).read_bytes()
    assert len(data) == row['bytes'] and hashlib.sha256(data).hexdigest() == row['sha256'], path
for name, contents in runpy.run_path(str(here/'prepare_reference.py'))['generated']().items():
    assert (here/name).read_text() == contents, name
proof = runpy.run_path(str(here/'transfer_proof.py'))
proof['legacy_force_stage']((root/'lib_src/constraints/tied_shell/runtime/CinForceStage.h').read_text())
proof['legacy_owner']((root/'lib_src/solvers/ExplicitNodalCinStep.cu').read_text())
body = proof['body']
helper = (root/'lib_src/solvers/cin_advance/ForceTransfers.h').read_text()
proof['same'](body(helper, 'Apply'), '''
  const auto force = force_inputs::ForceView(input);
  for (std::uint32_t row = 0; row < input.model.row_count; ++row) {
    const auto report = cin::detail::ApplyForceRow(input.model, force, row, input.prepared_transfers[row]);
    if (!report) return report;
  }
  return {};
''', 'ordered source rows and first error')
kernel = (root/'lib_src/solvers/cin_advance/ForceTransfers.cu').read_text()
leaf = body(kernel, 'Prepare')
assert leaf.index('input.control->status != NodalStatus::Ok') < leaf.index('force_inputs::ForceView')
assert 'row += gridDim.x*blockDim.x' in leaf
assert 'next.report = cin::detail::PrepareForceRow(input.model, force, row, next);' in leaf
assert 'input.prepared_transfers[row] = next;' in leaf
assert 'atomic' not in kernel and 'cudaMalloc' not in kernel
assert 'if (!input.model.row_count) return cudaSuccess;' in body(kernel, 'Launch')
owner = (root/'lib_src/solvers/ExplicitNodalCinStep.cu').read_text()
launch = body(owner, 'cin_advance::Launch')
assert launch.index('force_inputs::Launch') < launch.index('force_transfers::Launch') < launch.index('PrepareCin<<<')
assert launch.index('PrepareCin<<<') < launch.index('screen::Launch') < launch.index('AdvanceOrdinaryCin<<<')
assert 'parallel_inputs && input.prepared_transfers' in launch
assert 'cin->prepared_transfers}, stream)' in owner
startup = (root/'lib_src/solvers/NodalCinStartup.cpp').read_text()
assert 'if (next->dependent[row.secondary])' in startup
assert 'if (next->dependent[node])' in startup
assert 'Conflicting' not in leaf  # Authenticated topology is not reconstructed per attempt.
layout = (root/'lib_src/solvers/NodalCinLayout.h').read_text()
assert 'PreparedForceRow>(r, next.prepared_transfers)' in layout
storage = (root/'lib_src/solvers/NodalCinStorage.cu').read_text()
assert 'layout.prepared_transfers' in storage
runpy.run_path(str(here.parent/'cin_parallel_groups/verify_sources.py'))
print(json.dumps({'status':'passed', 'records':len(manifest['files']),
    'baseline':manifest['baseline_commit'], 'numerical_execution':False,
    'per_step_allocations':0, 'qualified_host_packet_bytes':512,
    'unchanged':['all node and source checks', 'source-row and repeated-slot application',
                 'first failure and partial force/coefficient fields', 'screen/motion/recovery/capture',
                 'accepted owner transaction']}))
