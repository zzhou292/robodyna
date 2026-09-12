#!/usr/bin/env python3
"""Authenticate the full frozen caller and exact legacy scalar extraction."""
from pathlib import Path
import hashlib
import json
import re
import runpy

here = Path(__file__).resolve().parent
root = here.parents[2]
raw = (here / 'source-manifest.json').read_bytes()
assert hashlib.sha256(raw).hexdigest() == 'd0480cd4749b486d8be97191ca34c038b2a2160b5cd4cdc2cbd2993036687f8f'
manifest = json.loads(raw)
for row in manifest['files']:
    path = Path(row['path'])
    assert not path.is_absolute() and '..' not in path.parts
    data = (root / path).read_bytes()
    assert len(data) == row['bytes'] and hashlib.sha256(data).hexdigest() == row['sha256'], path


def body(text, name):
    match = re.search(r'\b' + re.escape(name) + r'\s*\(', text)
    assert match, name
    start = text.index('{', match.start())
    end, depth = start + 1, 1
    while depth:
        depth += (text[end] == '{') - (text[end] == '}')
        end += 1
    return text[start + 1:end - 1]


def compact(text):
    return re.sub(r'\s+', '', text)


old = (here / 'frozen/ExplicitStepStability.h').read_text()
current = (root / 'lib_src/solvers/ExplicitStepStability.h').read_text()
original = body(old, 'FinalizeRows')
loop_start = original.index('  for(std::uint32_t i=0;')
scalar_start = original.index('  const double alpha=')
prefix = original[:loop_start].replace('StepLimit result;', 'result={};')
assert compact(body(current, 'BeginFinalizeRows')) == compact(prefix + 'return Status::kOk;')
assert body(current, 'CompleteFinalizeRows') == '\n' + original[scalar_start:]
wrapper = '''
  StepLimit result;
  const auto status=detail::BeginFinalizeRows(rows,safety,minimum_dt,requested_dt,out,result);
  if(status!=Status::kOk) return status;
''' + original[loop_start:scalar_start] + '''
  return detail::CompleteFinalizeRows(rows,safety,minimum_dt,result,out);
'''
assert compact(body(current, 'FinalizeRows')) == compact(wrapper)
assert old[:old.index('TL_SURFACE_HD inline Status FinalizeRows(')] == current[:current.index('namespace detail {')]

old_seal = (here / 'frozen/Validation.cuh').read_text()
seal = (root / 'lib_src/solvers/nodal_seal/Validation.cuh').read_text()
for name in ('Begin', 'FindFailure'):
    assert body(old_seal, name) == body(seal, name), name
old_finish = body(old_seal, 'Finish')
new_finish = body(seal, 'Finish')
begin = new_finish.index('  auto status = sc::Status::kOk;')
end = new_finish.index('  if (status != sc::Status::kOk)', begin)
legacy_call = '  const auto status = stability::FinalizeRows(&c->rows, safety, minimum_dt, h, &c->limit);\n'
assert old_finish == new_finish[:begin] + legacy_call + new_finish[end:]
rows = (root / 'lib_src/solvers/nodal_seal/Rows.cuh').read_text()
assert 'atomic' not in rows and 'cudaMalloc' not in rows
assert 'control->node != UINT32_MAX' in rows and '__syncthreads()' in rows
assert 'RowScratch rows = {}' in seal
assert 'rows.summaries != ControlTail(c,n).summaries' in seal
owner = (root / 'lib_src/solvers/FENodalState.cu').read_text()
forecast = (root / 'lib_src/solvers/NodalAssemblyCinForecast.cpp').read_text()
assert 'allocate(&next->control, layout.control.bytes)' in owner
assert 'sizeof(Control), cudaMemcpyDeviceToHost' in owner
assert 'nodal_seal::ControlBytes(sizeof(Control),n)' in owner
assert 'nodal_seal::ControlBytes(sizeof(Control),config.node_count)' in forecast
# Existing complete CIN source proofs stay active. Only reviewed owner budget
# and additive build entries are refreshed, with their prior records retained.
runpy.run_path(str(here.parent / 'cin_parallel_groups/verify_sources.py'))
print(json.dumps({'status': 'passed', 'baseline': manifest['baseline_commit'],
    'records': len(manifest['files']), 'maximum_device_tail_bytes': 8192,
    'per_step_allocations': 0, 'numerical_execution': False}))
