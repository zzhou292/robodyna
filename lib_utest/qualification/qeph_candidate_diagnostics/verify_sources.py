#!/usr/bin/env python3
"""Authenticate frozen diagnostics and unchanged material/ordered reduction bodies."""
from pathlib import Path
import hashlib,json
here=Path(__file__).resolve().parent
root=here.parents[2]
raw=(here/'source-manifest.json').read_bytes()
assert hashlib.sha256(raw).hexdigest()=="b7cc97f30982debee6973bcfabd7d99cb3ab6dae806018c3fff1dcc1ff3ac31d"
manifest=json.loads(raw)
for row in manifest['files']:
    path=Path(row['path'])
    assert not path.is_absolute() and '..' not in path.parts
    value=(root/path).read_bytes()
    assert len(value)==row['bytes'] and hashlib.sha256(value).hexdigest()==row['sha256'],path

def function(text,name):
    start=text.index(name+'(')
    begin=text.index('{',start)
    level=1
    end=begin+1
    while level:
        level+=(text[end]=='{')-(text[end]=='}')
        end+=1
    return text[start:end]
old=(here/'reference/QephBatchKernels.cu').read_text()
current=(root/'lib_src/elements/qeph/QephBatchKernels.cu').read_text()
for name in ('CandidateElements','FinalizeCandidate'):
    assert function(old,name)==function(current,name),name
old=(here/'reference/QephBatchDiagnostics.h').read_text()
current=(root/'lib_src/elements/qeph/QephBatchDiagnostics.h').read_text()
parent=function(current,'MeasureParents')
assert old[old.index('  unsigned material_parent=0;'):old.index('} // namespace')].strip()==parent[parent.index('  unsigned material_parent=0;'):].strip()
old_nodes=old[old.index('  auto& d=out.diagnostics;'):old.index('  unsigned material_parent=0;')]
assert old_nodes in function(current,'Measure')
print(json.dumps({'status':'passed','records':len(manifest['files']),'baseline':manifest['baseline_commit'],'unchanged_bodies':['CandidateElements','FinalizeCandidate','MeasureNodes','MeasureParents'],'numerical_execution':False}))
