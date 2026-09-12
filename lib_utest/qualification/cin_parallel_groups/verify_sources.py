#!/usr/bin/env python3
"""Authenticate exact serial baseline, checked extraction and all phase gates."""
from pathlib import Path
import hashlib
import json
import runpy

here = Path(__file__).resolve().parent
root = here.parents[2]
raw = (here/'source-manifest.json').read_bytes()
assert hashlib.sha256(raw).hexdigest() == 'ca5267d848c795ace51e8d1b0276d52c51c9eadb0bc9ca99d9312f5d677caab7'
manifest = json.loads(raw)
for row in manifest['files']:
    path = Path(row['path'])
    assert not path.is_absolute() and '..' not in path.parts
    data = (root/path).read_bytes()
    assert len(data) == row['bytes'] and hashlib.sha256(data).hexdigest() == row['sha256'], path
prepare = runpy.run_path(str(here/'prepare_reference.py'))
for name, data in prepare['generated']().items():
    assert (here/name).read_text() == data, name
proof = runpy.run_path(str(here/'group_proof.py'))
proof['legacy_owner']((root/'lib_src/solvers/ExplicitNodalCinStep.cu').read_text())
proof['legacy_values']((root/'lib_src/solvers/cin_timestep/ScreenValues.h').read_text())
proof['legacy_kernels']((root/'lib_src/solvers/cin_advance/Screen.cu').read_text())
body = proof['body']
helper = (root/'lib_src/solvers/cin_advance/Groups.h').read_text()
complete = body(helper, 'CompleteScreen')
assert complete.index('ordinary.invalid_node != UINT32_MAX') < complete.index('for (std::uint32_t group')
assert complete.index('if (value.visited)') < complete.index('if (value.status') < complete.index('Include(')
assert complete.index('Include(') < complete.index('output = next;')
motion = body(helper, 'AdvanceGroup')
assert motion.index('PrepareGroupCandidate<true>') < motion.index('for (std::uint32_t local')
assert 'PrepareGroupCandidate<false>' in motion and 'PrepareNodeOrientation' in motion
assert 'Report out{};' in motion and 'out.first_node = out.last_node = UINT32_MAX;' in motion
kernels = (root/'lib_src/solvers/cin_advance/Groups.cu').read_text()
assert 'atomic' not in kernels and 'cudaMalloc' not in kernels
launch = body(kernels, 'LaunchMotion')
assert launch.index('Begin<<<') < launch.index('Advance<<<') < launch.index('Complete<<<')
assert 'const auto failure = *input.failure;' in body(kernels, 'Begin')
assert 'group += gridDim.x*blockDim.x' in body(kernels, 'Advance')
assert 'group = 0; group < input.groups.group_count; ++group' in body(kernels, 'Complete')
assert 'input.control->limit.dt = 0;' in body(kernels, 'Fail')
screen = (root/'lib_src/solvers/cin_advance/Screen.cu').read_text()
launch = body(screen, 'Launch')
assert launch.index('Finish<<<') < launch.index('EvaluateGroups<<<') < launch.index('FinishGroups<<<')
assert 'input.screen[0].invalid_node != UINT32_MAX' in body(screen, 'EvaluateGroups')
layout = (root/'lib_src/solvers/NodalCinLayout.h').read_text()
assert 'group_count > n/2' in layout
assert 'device.Append<cin_advance::groups::Report>(group_count, next.group_reports)' in layout
assert 'sizeof(Report) == 24' in (root/'lib_src/solvers/cin_advance/GroupReport.h').read_text()
assert 'layout.group_reports.count' in (root/'lib_src/solvers/NodalCinStorage.cu').read_text()
assert 'ForecastCinStorage(*cin, c, cin_layout, group_count)' in (root/'lib_src/solvers/FENodalState.cu').read_text()
assert 'ForecastCinStorage(cin,config,attachment,binding.groups().size())' in (root/'lib_src/solvers/NodalAssemblyCinForecast.cpp').read_text()
# Existing force/ordinary/screen/capture proofs remain active on checked views.
runpy.run_path(str(here.parent/'cin_parallel_capture/verify_sources.py'))
print(json.dumps({'status':'passed', 'records':len(manifest['files']),
    'baseline':manifest['baseline_commit'], 'numerical_execution':False,
    'new_device_bytes_per_group':24, 'per_step_allocations':0,
    'unchanged':['within-group arithmetic and member order', 'CIN shared-master transfer',
                 'ordinary node motion', 'CIN recovery/drift', 'accepted owner transaction']}))
