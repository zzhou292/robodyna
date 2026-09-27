"""Authenticate frozen flow, shared numerical leaves and unchanged public preflight."""
from pathlib import Path
import hashlib,json,runpy
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[2]
manifest=json.loads((HERE/'source-manifest.json').read_text())
for row in manifest['files']:
    data=(ROOT/row['path']).read_bytes()
    assert len(data)==row['bytes'] and hashlib.sha256(data).hexdigest()==row['sha256'],row['path']
generator=runpy.run_path(str(HERE/'generate_oracle.py'))
assert (HERE/'SerialValues.h').read_text()==generator['generate']()
body=generator['body']
old=(HERE/'reference/T3BatchOnePointReadback.cpp.txt').read_text()
current=(ROOT/'lib_src/elements/t3/T3BatchOnePointReadback.cpp').read_text()
for name in ('ReadOnePoint','T3Batch::Impl::ValidateOnePointReadback'):
    assert body(old,name)==body(current,name),name
legacy=body(current,'T3Batch::Impl::ReadParentActivity')
legacy=legacy[legacy.index('  return shell_batch_plasticity_detail::ReadFailure'):]
assert legacy.strip()==body(old,'T3Batch::Impl::ReadParentActivity').strip()
old=(HERE/'reference/T3BatchFailureReadback.cpp.txt').read_text()
current=(ROOT/'lib_src/elements/t3/T3BatchFailureReadback.cpp').read_text()
for name in ('T3Batch::CopyAcceptedFailureHistory','T3Batch::CopyPreparedFailureHistory'):
    assert body(old,name)==body(current,name),name
for name in ('T3Batch::CopyAcceptedParentActivity','T3Batch::CopyPreparedParentActivity'):
    prior=body(old,name);now=body(current,name)
    boundary='  const auto report = state.ReadParentActivity' if 'Accepted' in name else '  const auto slab = 1u - state.AcceptedSlabIndex();'
    expected=prior[:prior.index(boundary)]
    boundary_now='  const std::uint8_t* compact_activity' if 'Accepted' in name else boundary
    assert now[:now.index(boundary_now)]==expected,name+' complete preflight'
read=(ROOT/'lib_src/elements/t3/mapped/ActivityReadback.cu').read_text()
order=['phase(Phase::OnePoint','phase(Phase::Mixed','phase(Phase::Failure','phase(Phase::Force','report=RoleAgreement','phase(Phase::PointIdentity']
locations=[read.index(token) for token in order];assert locations==sorted(locations)
assert 'ReadCompactActivity' in current or 'compact_activity' in current
kernel=(ROOT/'lib_src/elements/t3/mapped/ActivityKernels.cu').read_text()
assert 'atomicAdd' not in kernel and 'cudaMalloc' not in kernel
assert 'atomicMin(packet.first_invalid' in kernel
assert 'packet.active[parent] = 0' in kernel
values=(ROOT/'lib_src/elements/t3/mapped/ActivityValues.h').read_text()
for leaf in ('ValidOnePointState','FiniteSection','ValidFailureEncoding','ValidFailureState','ValidResult','one_point_detail::ValidValues','SameHistoryBits'):
    assert leaf in values,leaf
print(json.dumps({'status':'passed','baseline':manifest['baseline'],'frozen_files':len(manifest['files']),
    'scope':'frozen complete numerical loops, unchanged full-history/preflight and phase schedule','numerical_execution':False}))
