#!/usr/bin/env python3
"""Read-only T3 observer, frozen serial and unchanged mechanical body receipt."""
from pathlib import Path
import hashlib,json
here=Path(__file__).resolve().parent
root=here.parents[2]
raw=(here/'source-manifest.json').read_bytes()
assert hashlib.sha256(raw).hexdigest()=="0cb828e2eaf59aef9fb8b05143c4656dc3d429b90ac386a612936736c1321453"
manifest=json.loads(raw)
for row in manifest['files']:
    path=Path(row['path'])
    assert not path.is_absolute() and '..' not in path.parts
    data=(root/path).read_bytes()
    assert len(data)==row['bytes'] and hashlib.sha256(data).hexdigest()==row['sha256'],path
def body(text,name):
    begin=text.index('{',text.index(name+'('));end=begin+1;level=1
    while level:
        level+=(text[end]=='{')-(text[end]=='}');end+=1
    return text[begin:end]
def equal(value,name):
    assert hashlib.sha256(value.encode()).hexdigest()==manifest['baseline_bodies'][name],name
kernel=(root/'lib_src/elements/t3/T3BatchKernels.cu').read_text()
equal(body(kernel,'CandidateElements'),'CandidateElements')
measure=(root/'lib_src/elements/t3/T3BatchDiagnostics.h').read_text()
equal(body(measure,'Measure'),'Measure')
frozen=(here/'SerialDiagnostics.h').read_text()
equal(body(frozen,'Measure'),'Measure')
equal(body(frozen,'FinalizeCandidate').replace('t3_observer_test::frozen::Measure(', 'Measure('),'FinalizeCandidate')
assert 'if (mapped) LaunchMappedObserverDiagnostics(s, a, b, v, d, mixed);' in kernel
assert 'else FinalizeCandidate<<<1,1,0,v.stream>>>(s,a,b,v,d,mixed);' in kernel
print(json.dumps({'status':'passed','records':len(manifest['files']),
    'exact_bodies':['CandidateElements','Measure','frozen_complete_finalizer'],
    'numerical_execution':False}))
