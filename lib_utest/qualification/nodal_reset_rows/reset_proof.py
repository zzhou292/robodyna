"""Checked reset-only compatibility views for retained seal/CIN proofs."""
from pathlib import Path
import hashlib
import json
import re
import runpy

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
BASELINE = json.loads((HERE / 'baseline.json').read_text())


def region(text, name):
    match = re.search(r'\b' + re.escape(name) + r'\s*\(', text)
    assert match, name
    start = text.index('{', match.start())
    end, depth = start + 1, 1
    while depth:
        depth += (text[end] == '{') - (text[end] == '}')
        end += 1
    return start, end


def body(text, name):
    start, end = region(text, name)
    return text[start + 1:end - 1]


def same(a, b):
    assert re.sub(r'\s+', '', a) == re.sub(r'\s+', '', b)


def original(value, key):
    raw = value.encode()
    receipt = BASELINE['sources'][key]
    assert len(raw) == receipt['bytes']
    assert hashlib.sha256(raw).hexdigest() == receipt['sha256'], key
    return value


def legacy_stability(current):
    old = original((HERE / 'frozen/ExplicitStepStability.h').read_text(), 'stability')
    reset = body(old, 'ResetRows')
    loop = reset.index('  for(std::uint32_t')
    publish = reset.index('  rows->base_epoch=epoch;')
    same(body(current, 'BeginResetRows'), reset[:loop] + 'return Status::kOk;')
    same(body(current, 'CompleteResetRows'), reset[publish:])
    same(body(current, 'ResetRows'), '''
      const auto status = detail::BeginResetRows(rows, epoch, attempt);
      if (status != Status::kOk) return status;
    ''' + reset[loop:publish] + 'return detail::CompleteResetRows(rows, epoch, attempt);')
    first = old.index('TL_SURFACE_HD inline Status ResetRows(')
    old_end = region(old, 'ResetRows')[1]
    new_first = current.index('namespace detail {')
    new_end = region(current, 'ResetRows')[1]
    restored = current[:new_first] + old[first:old_end] + current[new_end:]
    assert restored == old
    return restored


def legacy_owner(current):
    kernel = (ROOT / 'lib_src/solvers/nodal_reset/Reset.cuh').read_text()
    old = (HERE / 'frozen/ResetTrial.cuh').read_text()
    prefix = body(old, 'ResetTrial').split('  if (stability::ResetRows(')[0]
    same(body(kernel, 'ResetTrial'), '''
      __shared__ tlfea::contact::Status reset_status;
      if (threadIdx.x == 0) {
    ''' + prefix + '''
        reset_status = stability::detail::BeginResetRows(&c->rows, epoch, attempt);
        if (reset_status != tlfea::contact::Status::kOk)
          c->status = NodalStatus::InvalidOutput;
      }
      __syncthreads();
      // Every lane sees the same result and takes the same rejected return.
      if (reset_status != tlfea::contact::Status::kOk) return;
      for (std::size_t node = threadIdx.x; node < c->rows.node_count; node += blockDim.x) {
        c->rows.stiffness[node] = 0;
        c->rows.damping[node] = 0;
      }
      __syncthreads();
      if (threadIdx.x == 0)
        stability::detail::CompleteResetRows(&c->rows, epoch, attempt);
    ''')
    assert 'inline constexpr unsigned Threads = 256;' in kernel
    include = '#include "nodal_reset/Reset.cuh"\n'
    launch = 'nodal_reset::ResetTrial<<<1,nodal_reset::Threads,0,s.stream>>>'
    anchor = '}  // namespace\n\nFENodalState::FENodalState()'
    assert current.count(include) == current.count(launch) == current.count(anchor) == 1
    restored = current.replace(include, '').replace(launch, 'ResetTrial<<<1,1,0,s.stream>>>')
    restored = restored.replace(anchor, old + anchor)
    return original(restored, 'owner')


def legacy_build(current):
    # Reverse the independently checked cooperative header registration first;
    # retain the exact old drift/reset inverses and final whole-file digest.
    if '"cin_advance/GroupResponse.h"' in current:
        cooperative = runpy.run_path(str(HERE.parent / 'cin_cooperative_group_screen/cooperative_proof.py'))
        current = cooperative['legacy_build'](current)
    addition = ' "nodal_reset/Reset.cuh",'
    assert current.count(addition) == 1
    restored = current.replace(addition, '')
    # Independent scheduling slices share the original owner target. Reverse
    # the authenticated drift source addition before checking our common base.
    if '"cin_advance/Drift.cu"' in restored:
        drift = runpy.run_path(str(HERE.parent / 'cin_parallel_drift/drift_proof.py'))
        restored = drift['legacy_build'](restored)
    return original(restored, 'build')
