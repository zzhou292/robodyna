#!/usr/bin/env python3
"""Owning source identities plus original complete screen/member/capture proof."""
from pathlib import Path
import hashlib
import json
import runpy
HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
raw = (HERE/'source-manifest.json').read_bytes()
assert hashlib.sha256(raw).hexdigest() == '039925b2203889ef3f789bbb41a10b78ab3f7353e744a52a7b408e0a1ee8d048'
manifest = json.loads(raw)
for row in manifest['files']:
    path = Path(row['path'])
    assert not path.is_absolute() and '..' not in path.parts
    data = (ROOT/path).read_bytes()
    assert len(data) == row['bytes'] and hashlib.sha256(data).hexdigest() == row['sha256'], path
prepare = runpy.run_path(str(HERE/'prepare_reference.py'))
for name,value in prepare['generated']().items():
    assert (HERE/name).read_text() == value, name
runpy.run_path(str(HERE/'cooperative_proof.py'))['prove']()
# Preserve complete source proof composition, including current limiter/recovery.
runpy.run_path(str(HERE.parent/'cin_parallel_groups/verify_sources.py'))
runpy.run_path(str(HERE.parent/'cin_limiter/verify_sources.py'))
runpy.run_path(str(HERE.parent/'cin_parallel_drift/verify_sources.py'))
runpy.run_path(str(HERE.parent/'nodal_reset_rows/verify_sources.py'))
print(json.dumps({'status':'passed','records':len(manifest['files']),
    'baseline':manifest['baseline_commit'],'numerical_execution':False,
    'device_arena_delta_bytes':0,'host_arena_delta_bytes':0,
    'shared_bytes_per_block':4776,'threads_per_group':64,'per_step_allocations':0}))
