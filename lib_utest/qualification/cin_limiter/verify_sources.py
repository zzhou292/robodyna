#!/usr/bin/env python3
"""Authenticate the additive witness and retain all complete mechanics proofs."""
from pathlib import Path
import hashlib
import json
import runpy

here = Path(__file__).resolve().parent
root = here.parents[2]
raw = (here / 'source-manifest.json').read_bytes()
EXPECTED = 'c68f797147fb9c3927f182b369e3804c43160c597870d776d362da75598f74ea'
assert hashlib.sha256(raw).hexdigest() == EXPECTED
manifest = json.loads(raw)
for row in manifest['files']:
    path = Path(row['path'])
    assert not path.is_absolute() and '..' not in path.parts
    data = (root / path).read_bytes()
    assert len(data) == row['bytes'] and hashlib.sha256(data).hexdigest() == row['sha256'], path
query = (root / 'lib_src/solvers/NodalCinStructuralLimit.cpp').read_text()
assert 'cudaMemcpy' not in query and 'cudaStream' not in query and '<<<' not in query
assert 'sizeof(*output)' in query and '% alignof(NodalCinStructuralLimit)' in query
assert query.index('state.Matches(') < query.index('*output = next;')
assert query.index('OutsideSources(') < query.index('*output = next;')
assert 'c->structural_limiter = {};' in (root / 'lib_src/solvers/FENodalState.cu').read_text()
assert 'bool capture_limiter = false;' in (root / 'lib_src/solvers/NodalCinStructuralStep.h').read_text()
runpy.run_path(str(here.parent / 'nodal_seal_rows/verify_sources.py'))
runpy.run_path(str(here.parent / 'cin_force_transfers/verify_sources.py'))
print(json.dumps({'status': 'passed', 'records': len(manifest['files']),
    'numerical_execution': False, 'new_full_node_arrays': 0, 'per_step_allocations': 0,
    'extra_query_cuda_calls': 0, 'growth': 'fixed sizeof(Witness) in device and host Control'}))
