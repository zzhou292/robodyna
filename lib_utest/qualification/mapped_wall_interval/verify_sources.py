#!/usr/bin/env python3
"""Authenticate complete serial authority, interval proof and mapped-only seam."""
from pathlib import Path
import hashlib, json, re, subprocess, sys
here = Path(__file__).resolve().parent
root = here.parents[2]
raw = (here/'source-manifest.json').read_bytes()
assert hashlib.sha256(raw).hexdigest() == "70f76b140dca9c2725f3d25f454a6df2337359f595c76375449d36140518aa91"
manifest = json.loads(raw)
for row in manifest['files']:
    path = Path(row['path'])
    assert not path.is_absolute() and '..' not in path.parts
    data = (root/path).read_bytes()
    assert len(data) == row['bytes'] and hashlib.sha256(data).hexdigest() == row['sha256'], path
original = (here/'reference/NodalWallContactDiagnostics.h').read_text()
current = (root/'lib_src/collision/NodalWallContactDiagnostics.h').read_text()
frozen = (here/'SerialInterval.h').read_text()
assert original == current

def body(text, name):
    begin = text.index('{', text.index(name+'('))
    end, depth = begin+1, 1
    while depth:
        depth += (text[end] == '{')-(text[end] == '}')
        end += 1
    return text[begin:end]

assert body(original, 'MeasureInterval') == body(frozen, 'MeasureInterval')
# Whole post-loop sequence copied in original order; only failure routing differs.
post = body(original, 'MeasureInterval')
post = post[post.index('  if (!Radius(d.kick_work'):]
post = post.replace('return Fail(s.control,Code::NonFiniteArithmetic);', 'return false;')
apply = body((root/'lib_src/collision/nodal_wall_mapped/IntervalFinalizer.h').read_text(), 'ApplyInterval')
apply = apply[apply.index('  if (!Radius(d.kick_work'):]
assert re.sub(r'\s+', '', post) == re.sub(r'\s+', '', apply)
operations = (root/'lib_src/collision/nodal_wall_mapped/Operations.cu').read_text()
sys.path.insert(0, str(here.parent/'mapped_wall_removal_events'))
from removal_proof import legacy_operations
operations = legacy_operations(operations)
assert operations.count('parallel::MeasureInterval(') == 1
assert '{state.remote.interval,state.layout.interval.count}' in operations
assert 'd::MeasureInterval(*storage,view)' not in operations
assert 'm::RemovedPotential(storage,side)' in operations
assert 'Construct<m::IntervalSummary>(layout.interval)' in (root/'lib_src/collision/nodal_wall_mapped/Initialize.cpp').read_text()
subprocess.run([sys.executable, '-B', str(here.parent/'mapped_wall_observers/verify_sources.py')], check=True)
print(json.dumps({'status':'passed','records':len(manifest['files']),
    'unchanged':['complete_serial_interval','rounded_leaf_math','physical_mechanics','legacy_default','removed_potential'],
    'numerical_execution':False}))
