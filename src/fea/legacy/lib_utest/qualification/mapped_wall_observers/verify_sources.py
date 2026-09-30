#!/usr/bin/env python3
"""Authenticate the frozen serial fold, opt-in seam and unchanged mechanics."""
from pathlib import Path
import hashlib,json
here=Path(__file__).resolve().parent
root=here.parents[2]
raw=(here/'source-manifest.json').read_bytes()
assert hashlib.sha256(raw).hexdigest() == "5d5b14a036bbdf08e568c090946667ba88364713a3a49919a6a3b6cedc3382c8"
manifest=json.loads(raw)
for row in manifest['files']:
    path=Path(row['path'])
    assert not path.is_absolute() and '..' not in path.parts
    value=(root/path).read_bytes()
    assert len(value)==row['bytes'] and hashlib.sha256(value).hexdigest()==row['sha256'],path
def body(text,name):
    begin=text.index('{',text.index(name+'('));end=begin+1;depth=1
    while depth:
        depth+=(text[end]=='{')-(text[end]=='}');end+=1
    return text[begin:end]
original=(here/'reference/NodalWallContactEvaluation.cuh').read_text()
current=(root/'lib_src/collision/NodalWallContactEvaluation.cuh').read_text()
frozen=(here/'SerialObservers.h').read_text()
assert original==current
assert body(original,'ReduceNodes')==body(frozen,'ReduceNodes')
# Existing whole physical point/parent oracle remains independently frozen.
old=json.loads((here.parent/'mapped_wall_evaluation/source-manifest.json').read_bytes())
for row in old['files']:
    assert hashlib.sha256((root/row['path']).read_bytes()).hexdigest()==row['sha256'],row['path']
seam=(root/'lib_src/collision/nodal_wall_mapped/Evaluation.cuh').read_text()
assert 'ObserverScratch observers={}' in seam and 'else Finish<<<1,1,0,stream>>>' in seam
operations=(root/'lib_src/collision/nodal_wall_mapped/Operations.cu').read_text()
assert operations.count('{state.remote.observer,state.layout.observer.count}')==2
assert 'Construct<m::ObserverSummary>(layout.observer)' in (root/'lib_src/collision/nodal_wall_mapped/Initialize.cpp').read_text()
print(json.dumps({'status':'passed','records':len(manifest['files']),
    'unchanged':['physical_point','parent_certificates','response','scatter','MeasureInterval','serial_fallback'],
    'numerical_execution':False}))
