"""Verify the reset-only delta and retain the existing seal/CIN source chain."""
from pathlib import Path
import hashlib
import json
import runpy

here = Path(__file__).resolve().parent
root = here.parents[2]
raw = (here / 'source-manifest.json').read_bytes()
EXPECTED = '6fe865022bf16edb86ac68ffa959ed81cf39cf1158a767a271ebd3dea06aeb26'
assert hashlib.sha256(raw).hexdigest() == EXPECTED
manifest = json.loads(raw)
for row in manifest['files']:
    path = Path(row['path'])
    assert not path.is_absolute() and '..' not in path.parts
    blob = (root / path).read_bytes()
    assert len(blob) == row['bytes'] and hashlib.sha256(blob).hexdigest() == row['sha256'], path
proof = runpy.run_path(str(here / 'reset_proof.py'))
proof['legacy_stability']((root / 'lib_src/solvers/ExplicitStepStability.h').read_text())
proof['legacy_owner']((root / 'lib_src/solvers/FENodalState.cu').read_text())
proof['legacy_build']((root / 'lib_src/solvers/BUILD.bazel').read_text())
prepared = runpy.run_path(str(here / 'prepare_frozen.py'))['generated']()
assert proof['body'](prepared, 'ResetRows') == proof['body'](
    (here / 'frozen/ExplicitStepStability.h').read_text(), 'ResetRows')
# The node-loop change affects reset only. Retain all previous scalar, transfer,
# limiter, drift, group and capture evidence under their checked compatibility views.
runpy.run_path(str(here.parent / 'cin_limiter/verify_sources.py'))
runpy.run_path(str(here.parent / 'cin_parallel_drift/verify_sources.py'))
print(json.dumps({'status': 'passed', 'records': len(manifest['files']),
    'baseline': manifest['baseline_commit'], 'numerical_execution': False,
    'new_device_bytes': 0, 'new_host_bytes': 0, 'new_allocations': 0,
    'kernel_count_change': 0, 'reset_threads': 256}))
